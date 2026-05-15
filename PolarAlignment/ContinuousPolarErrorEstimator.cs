using NINA.Astrometry;
using NINA.Core.Utility;
using NINA.PlateSolving;
using System;
using System.Windows;

namespace NINA.Plugins.PolarAlignment {
    /// <summary>
    /// Estimates the remaining polar-alignment error during the continuous correction phase.
    ///
    /// High-level idea:
    /// 1. The initial three-point routine measures the mount's starting polar-axis error.
    /// 2. During manual/automated adjustment, each new plate solve gives the current field center.
    /// 3. This class asks: "Which residual azimuth/altitude polar-axis error would predict that current field center?"
    /// 4. It solves that inverse problem with a damped nonlinear least-squares step in a 2D state space.
    ///
    /// The state vector is:
    ///     e = [ residualAzimuthErrorDegrees, residualAltitudeErrorDegrees ]
    ///
    /// The observation is the current plate-solved field center, expressed in apparent topocentric coordinates
    /// at the actual observation time. The forward model reuses the plugin's physical approximation that
    /// polar-axis adjustment can be represented locally as:
    ///     1. a rotation around the local vertical axis (azimuth adjustment)
    ///     2. followed by a rotation around the corresponding horizontal axis (altitude adjustment)
    ///
    /// Compared with the old image-plane heuristic, this estimator works directly in angular space and uses
    /// the plate-solved field center as the measurement, while the overlay becomes visualization only.
    /// </summary>
    internal static class ContinuousPolarErrorEstimator {
        // Finite-difference step used to estimate the local Jacobian numerically.
        // 1/720 degree = 5 arcseconds. This is small enough to approximate the derivative cleanly
        // without becoming dominated by floating-point cancellation.
        private const double DerivativeStepDegrees = 1.0 / 720.0;

        // The solve is tiny (2 unknowns), so a small fixed iteration cap is enough.
        private const int MaxIterations = 20;

        // Reject near-singular systems. This is a safeguard against geometries where one parameter
        // becomes effectively unobservable from the current field-center measurement.
        private const double MinimumEigenvalue = 1e-12;

        // If the normal matrix becomes too ill-conditioned, the estimate is considered unstable.
        private const double MaximumConditionNumber = 1e6;

        // Stop when the update is already much smaller than any practically meaningful correction.
        // 1/360000 degree ~= 0.01 arcseconds.
        private const double ConvergenceThresholdDegrees = 1.0 / 360000.0;

        internal sealed class EstimationResult {
            public EstimationResult(bool success,
                                    double azimuthErrorDegrees,
                                    double altitudeErrorDegrees,
                                    double conditionNumber,
                                    double residualArcSeconds) {
                Success = success;
                AzimuthErrorDegrees = azimuthErrorDegrees;
                AltitudeErrorDegrees = altitudeErrorDegrees;
                ConditionNumber = conditionNumber;
                ResidualArcSeconds = residualArcSeconds;
            }

            public bool Success { get; }
            public double AzimuthErrorDegrees { get; }
            public double AltitudeErrorDegrees { get; }
            public double ConditionNumber { get; }
            public double ResidualArcSeconds { get; }
        }

        // TopocentricCoordinates needs an ICustomDateTime instance. During the continuous solve
        // we want to keep the exact observation timestamp fixed, so we wrap it here.
        private sealed class FixedDateTime : ICustomDateTime {
            private readonly DateTime dateTime;

            public FixedDateTime(DateTime dateTime) {
                this.dateTime = dateTime;
            }

            public DateTime Now => dateTime;

            public DateTime UtcNow => dateTime.Kind == DateTimeKind.Utc
                ? dateTime
                : dateTime.ToUniversalTime();
        }

        internal static EstimationResult Estimate(PolarErrorDetermination determination,
                                                  PlateSolveResult currentReferenceFrame,
                                                  RefractionParameters refractionParameters,
                                                  double seedAzimuthErrorDegrees,
                                                  double seedAltitudeErrorDegrees) {
            if (determination == null || currentReferenceFrame?.Coordinates == null) {
                return new EstimationResult(false, seedAzimuthErrorDegrees, seedAltitudeErrorDegrees, double.PositiveInfinity, double.PositiveInfinity);
            }

            // The current plate solve is the measurement. Convert it to apparent topocentric coordinates
            // using the actual current timestamp, location, elevation, and refraction parameters.
            var observationTime = currentReferenceFrame.Coordinates.DateTime.Now;
            var observedTopocentric = TransformToTopocentric(currentReferenceFrame.Coordinates,
                                                             determination.Latitude,
                                                             determination.Longitude,
                                                             determination.Elevation,
                                                             refractionParameters,
                                                             observationTime);

            // Seed from the previously estimated residual. The correction loop is continuous, so the
            // new solution should usually be close to the last one and converge quickly.
            var state = new[] { seedAzimuthErrorDegrees, seedAltitudeErrorDegrees };

            // Levenberg-Marquardt style damping. Small lambda behaves like Gauss-Newton, larger lambda
            // damps the step when the local linearization is not reliable enough.
            var lambda = 1e-3d;
            var currentCost = EvaluateCost(determination,
                                           observedTopocentric,
                                           refractionParameters,
                                           observationTime,
                                           state[0],
                                           state[1]);

            var conditionNumber = double.PositiveInfinity;
            var residualArcSeconds = double.PositiveInfinity;

            for (var iteration = 0; iteration < MaxIterations; iteration++) {
                // Build the 2x2 normal equations J^T J and J^T r for the current linearization point.
                if (!TryBuildNormalEquations(determination,
                                             observedTopocentric,
                                             refractionParameters,
                                             observationTime,
                                             state[0],
                                             state[1],
                                             out var a00,
                                             out var a01,
                                             out var a11,
                                             out var gradient0,
                                             out var gradient1,
                                             out conditionNumber,
                                             out residualArcSeconds)) {
                    return new EstimationResult(false, state[0], state[1], conditionNumber, residualArcSeconds);
                }

                // Solve (J^T J + lambda I) delta = -J^T r
                if (!TrySolve2x2(a00 + lambda, a01, a11 + lambda, -gradient0, -gradient1, out var deltaAzimuth, out var deltaAltitude)) {
                    return new EstimationResult(false, state[0], state[1], conditionNumber, residualArcSeconds);
                }

                // If the proposed update is already negligible, the nonlinear solve has converged.
                if (Math.Max(Math.Abs(deltaAzimuth), Math.Abs(deltaAltitude)) < ConvergenceThresholdDegrees) {
                    break;
                }

                var accepted = false;
                var lineSearchScale = 1.0d;

                while (lineSearchScale >= 0.0625d) {
                    // Backtracking line search: only accept a step if the true nonlinear cost improves.
                    var candidateAzimuth = state[0] + deltaAzimuth * lineSearchScale;
                    var candidateAltitude = state[1] + deltaAltitude * lineSearchScale;
                    var candidateCost = EvaluateCost(determination,
                                                     observedTopocentric,
                                                     refractionParameters,
                                                     observationTime,
                                                     candidateAzimuth,
                                                     candidateAltitude);

                    if (candidateCost < currentCost) {
                        state[0] = candidateAzimuth;
                        state[1] = candidateAltitude;
                        currentCost = candidateCost;

                        // A successful step means we can trust the local quadratic model a bit more.
                        lambda = Math.Max(lambda * 0.5d, 1e-6d);
                        accepted = true;
                        break;
                    }

                    lineSearchScale *= 0.5d;
                }

                if (!accepted) {
                    // If no candidate improved the cost, fall back to heavier damping.
                    lambda *= 8d;
                    if (lambda > 1e6d) {
                        break;
                    }
                }
            }

            if (!TryBuildNormalEquations(determination,
                                         observedTopocentric,
                                         refractionParameters,
                                         observationTime,
                                         state[0],
                                         state[1],
                                         out _,
                                         out _,
                                         out _,
                                         out _,
                                         out _,
                                         out conditionNumber,
                                         out residualArcSeconds)) {
                return new EstimationResult(false, state[0], state[1], conditionNumber, residualArcSeconds);
            }

            return new EstimationResult(conditionNumber <= MaximumConditionNumber,
                                        state[0],
                                        state[1],
                                        conditionNumber,
                                        residualArcSeconds);
        }

        /// <summary>
        /// Forward model in topocentric space.
        ///
        /// Starting from the initial reference-frame center coordinate, predict where that same field center
        /// would appear if the mount currently had the supplied residual polar-axis error.
        ///
        /// The key relation is that we do not rotate by the residual error directly. We rotate by the change
        /// from the initial measured error:
        ///     deltaAz  = initialAz  - residualAz
        ///     deltaAlt = initialAlt - residualAlt
        ///
        /// This makes the endpoints behave correctly:
        /// - residual == initial  => zero physical correction applied
        /// - residual == 0        => full correction applied
        /// </summary>
        internal static TopocentricCoordinates GetTopocentricCoordinateForResidual(Coordinates coordinate,
                                                                                   double initialAzimuthErrorDegrees,
                                                                                   double initialAltitudeErrorDegrees,
                                                                                   double residualAzimuthErrorDegrees,
                                                                                   double residualAltitudeErrorDegrees,
                                                                                   Angle latitude,
                                                                                   Angle longitude,
                                                                                   double elevation,
                                                                                   RefractionParameters refractionParameters,
                                                                                   DateTime observationTime) {
            var referenceTopocentric = TransformToTopocentric(coordinate,
                                                              latitude,
                                                              longitude,
                                                              elevation,
                                                              refractionParameters,
                                                              observationTime);

            var referenceVector = Vector3.CoordinatesToUnitVector(referenceTopocentric);
            var azimuthRotation = Angle.ByDegree(initialAzimuthErrorDegrees - residualAzimuthErrorDegrees);
            var altitudeRotation = Angle.ByDegree(initialAltitudeErrorDegrees - residualAltitudeErrorDegrees);

            // First rotate around the local vertical axis. This models azimuth adjustment.
            var azimuthAdjusted = Vector3.RotateByRodrigues(referenceVector, new Vector3(0, 0, 1), azimuthRotation);

            // Then rotate around the horizontal axis associated with the new azimuth. This models altitude
            // adjustment after the azimuth change has already been applied.
            var altitudeAxis = Vector3.RotateByRodrigues(new Vector3(0, 1, 0), new Vector3(0, 0, 1), azimuthRotation);
            var correctedVector = Vector3.RotateByRodrigues(azimuthAdjusted, altitudeAxis, altitudeRotation);

            return correctedVector.ToTopocentric(latitude, longitude, elevation, new FixedDateTime(observationTime));
        }

        /// <summary>
        /// Same forward model as GetTopocentricCoordinateForResidual, but converted back into equatorial
        /// coordinates so the result can be projected into the current image using the plate-solve WCS.
        /// This is used by the on-screen overlay and by tests.
        /// </summary>
        internal static Coordinates GetCoordinateForResidual(Coordinates coordinate,
                                                             double initialAzimuthErrorDegrees,
                                                             double initialAltitudeErrorDegrees,
                                                             double residualAzimuthErrorDegrees,
                                                             double residualAltitudeErrorDegrees,
                                                             Angle latitude,
                                                             Angle longitude,
                                                             double elevation,
                                                             RefractionParameters refractionParameters,
                                                             DateTime observationTime) {
            var correctedTopocentric = GetTopocentricCoordinateForResidual(coordinate,
                                                                           initialAzimuthErrorDegrees,
                                                                           initialAltitudeErrorDegrees,
                                                                           residualAzimuthErrorDegrees,
                                                                           residualAltitudeErrorDegrees,
                                                                           latitude,
                                                                           longitude,
                                                                           elevation,
                                                                           refractionParameters,
                                                                           observationTime);

            refractionParameters = refractionParameters ?? RefractionParameters.GetRefractionParameters();
            return correctedTopocentric.Transform(Epoch.J2000,
                                                  refractionParameters.PressureHPa,
                                                  refractionParameters.Temperature,
                                                  refractionParameters.RelativeHumidity,
                                                  refractionParameters.Wavelength);
        }

        /// <summary>
        /// Projects a predicted sky coordinate into the current image frame.
        /// The estimate itself works in angular space, but the display overlay still needs image pixels.
        /// </summary>
        internal static Point ProjectCoordinateForResidual(Coordinates coordinate,
                                                           PlateSolveResult currentReferenceFrame,
                                                           Point imageCenter,
                                                           double arcsecPerPix,
                                                           double initialAzimuthErrorDegrees,
                                                           double initialAltitudeErrorDegrees,
                                                           double residualAzimuthErrorDegrees,
                                                           double residualAltitudeErrorDegrees,
                                                           Angle latitude,
                                                           Angle longitude,
                                                           double elevation,
                                                           RefractionParameters refractionParameters) {
            var observationTime = currentReferenceFrame.Coordinates.DateTime.Now;
            var predictedCoordinates = GetCoordinateForResidual(coordinate,
                                                                initialAzimuthErrorDegrees,
                                                                initialAltitudeErrorDegrees,
                                                                residualAzimuthErrorDegrees,
                                                                residualAltitudeErrorDegrees,
                                                                latitude,
                                                                longitude,
                                                                elevation,
                                                                refractionParameters,
                                                                observationTime);

            return predictedCoordinates.XYProjection(currentReferenceFrame.Coordinates,
                                                    imageCenter,
                                                    arcsecPerPix,
                                                    arcsecPerPix,
                                                    currentReferenceFrame.PositionAngle);
        }

        /// <summary>
        /// Shared helper that converts equatorial coordinates to apparent topocentric coordinates with
        /// the exact same refraction/time convention everywhere in the continuous estimator.
        /// </summary>
        private static TopocentricCoordinates TransformToTopocentric(Coordinates coordinate,
                                                                     Angle latitude,
                                                                     Angle longitude,
                                                                     double elevation,
                                                                     RefractionParameters refractionParameters,
                                                                     DateTime observationTime) {
            refractionParameters = refractionParameters ?? RefractionParameters.GetRefractionParameters();
            return coordinate.Transform(latitude,
                                        longitude,
                                        elevation,
                                        refractionParameters.PressureHPa,
                                        refractionParameters.Temperature,
                                        refractionParameters.RelativeHumidity,
                                        refractionParameters.Wavelength,
                                        observationTime);
        }

        /// <summary>
        /// Nonlinear objective function:
        ///     C(e) = 1/2 * || r(e) ||^2
        ///
        /// where r(e) is the 2D residual in local angular coordinates.
        /// </summary>
        private static double EvaluateCost(PolarErrorDetermination determination,
                                           TopocentricCoordinates observedTopocentric,
                                           RefractionParameters refractionParameters,
                                           DateTime observationTime,
                                           double azimuthErrorDegrees,
                                           double altitudeErrorDegrees) {
            if (!TryEvaluateResidual(determination,
                                     observedTopocentric,
                                     refractionParameters,
                                     observationTime,
                                     azimuthErrorDegrees,
                                     altitudeErrorDegrees,
                                     out var residualAzimuth,
                                     out var residualAltitude)) {
                return double.PositiveInfinity;
            }

            return 0.5d * (residualAzimuth * residualAzimuth + residualAltitude * residualAltitude);
        }

        /// <summary>
        /// Builds the 2x2 normal equations used by the Gauss-Newton / LM step.
        ///
        /// Because the state is only [az, alt], the Jacobian is 2x2 and we can compute everything directly:
        ///     A = J^T J
        ///     g = J^T r
        ///
        /// The same routine also estimates conditioning from the eigenvalues of A.
        /// </summary>
        private static bool TryBuildNormalEquations(PolarErrorDetermination determination,
                                                    TopocentricCoordinates observedTopocentric,
                                                    RefractionParameters refractionParameters,
                                                    DateTime observationTime,
                                                    double azimuthErrorDegrees,
                                                    double altitudeErrorDegrees,
                                                    out double a00,
                                                    out double a01,
                                                    out double a11,
                                                    out double gradient0,
                                                    out double gradient1,
                                                    out double conditionNumber,
                                                    out double residualArcSeconds) {
            a00 = 0;
            a01 = 0;
            a11 = 0;
            gradient0 = 0;
            gradient1 = 0;
            conditionNumber = double.PositiveInfinity;
            residualArcSeconds = double.PositiveInfinity;

            if (!TryEvaluateResidual(determination,
                                     observedTopocentric,
                                     refractionParameters,
                                     observationTime,
                                     azimuthErrorDegrees,
                                     altitudeErrorDegrees,
                                     out var residualAzimuth,
                                     out var residualAltitude)) {
                return false;
            }

            var derivativeAzimuth = DifferentiateResidual(determination,
                                                          observedTopocentric,
                                                          refractionParameters,
                                                          observationTime,
                                                          azimuthErrorDegrees,
                                                          altitudeErrorDegrees,
                                                          true);
            var derivativeAltitude = DifferentiateResidual(determination,
                                                           observedTopocentric,
                                                           refractionParameters,
                                                           observationTime,
                                                           azimuthErrorDegrees,
                                                           altitudeErrorDegrees,
                                                           false);

            // Assemble J^T J and J^T r explicitly for the two-parameter system.
            a00 = derivativeAzimuth.X * derivativeAzimuth.X + derivativeAzimuth.Y * derivativeAzimuth.Y;
            a01 = derivativeAzimuth.X * derivativeAltitude.X + derivativeAzimuth.Y * derivativeAltitude.Y;
            a11 = derivativeAltitude.X * derivativeAltitude.X + derivativeAltitude.Y * derivativeAltitude.Y;
            gradient0 = derivativeAzimuth.X * residualAzimuth + derivativeAzimuth.Y * residualAltitude;
            gradient1 = derivativeAltitude.X * residualAzimuth + derivativeAltitude.Y * residualAltitude;
            residualArcSeconds = Math.Sqrt(residualAzimuth * residualAzimuth + residualAltitude * residualAltitude) * 3600d;

            // Conditioning check:
            // - if the smaller eigenvalue is tiny, the system is nearly singular
            // - if eigenMax / eigenMin is huge, the estimate is overly sensitive to noise
            var trace = a00 + a11;
            var discriminant = Math.Sqrt(Math.Max(0d, (a00 - a11) * (a00 - a11) + 4d * a01 * a01));
            var eigenMax = 0.5d * (trace + discriminant);
            var eigenMin = 0.5d * (trace - discriminant);

            if (eigenMin <= MinimumEigenvalue || !double.IsFinite(eigenMin) || !double.IsFinite(eigenMax)) {
                return false;
            }

            conditionNumber = eigenMax / eigenMin;
            return double.IsFinite(conditionNumber);
        }

        /// <summary>
        /// Numerical Jacobian of the residual with respect to the chosen parameter.
        ///
        /// We use centered differences:
        ///     dr/dx ~= (r(x+h) - r(x-h)) / (2h)
        ///
        /// The residual function is smooth and low-dimensional, so this is accurate and simple.
        /// </summary>
        private static Vector DifferentiateResidual(PolarErrorDetermination determination,
                                                    TopocentricCoordinates observedTopocentric,
                                                    RefractionParameters refractionParameters,
                                                    DateTime observationTime,
                                                    double azimuthErrorDegrees,
                                                    double altitudeErrorDegrees,
                                                    bool azimuthDerivative) {
            var positive = azimuthDerivative
                ? EvaluateResidualForDerivative(determination,
                                                observedTopocentric,
                                                refractionParameters,
                                                observationTime,
                                                azimuthErrorDegrees + DerivativeStepDegrees,
                                                altitudeErrorDegrees)
                : EvaluateResidualForDerivative(determination,
                                                observedTopocentric,
                                                refractionParameters,
                                                observationTime,
                                                azimuthErrorDegrees,
                                                altitudeErrorDegrees + DerivativeStepDegrees);

            var negative = azimuthDerivative
                ? EvaluateResidualForDerivative(determination,
                                                observedTopocentric,
                                                refractionParameters,
                                                observationTime,
                                                azimuthErrorDegrees - DerivativeStepDegrees,
                                                altitudeErrorDegrees)
                : EvaluateResidualForDerivative(determination,
                                                observedTopocentric,
                                                refractionParameters,
                                                observationTime,
                                                azimuthErrorDegrees,
                                                altitudeErrorDegrees - DerivativeStepDegrees);

            return (positive - negative) / (2d * DerivativeStepDegrees);
        }

        private static Vector EvaluateResidualForDerivative(PolarErrorDetermination determination,
                                                            TopocentricCoordinates observedTopocentric,
                                                            RefractionParameters refractionParameters,
                                                            DateTime observationTime,
                                                            double azimuthErrorDegrees,
                                                            double altitudeErrorDegrees) {
            if (!TryEvaluateResidual(determination,
                                     observedTopocentric,
                                     refractionParameters,
                                     observationTime,
                                     azimuthErrorDegrees,
                                     altitudeErrorDegrees,
                                     out var residualAzimuth,
                                     out var residualAltitude)) {
                return new Vector(double.NaN, double.NaN);
            }

            return new Vector(residualAzimuth, residualAltitude);
        }

        /// <summary>
        /// Residual definition for the nonlinear solve.
        ///
        /// The observation and prediction are both in apparent topocentric coordinates.
        /// The residual is expressed in a local small-angle metric:
        ///     r = [ dAz * cos(meanAlt), dAlt ]
        ///
        /// Why scale azimuth by cos(alt)?
        /// A one-degree change in azimuth does not correspond to a one-degree angular displacement on the sky
        /// except at the horizon. Near the zenith, the same azimuth difference corresponds to less motion.
        /// Multiplying by cos(alt) converts azimuth degrees into the same first-order local metric as altitude.
        /// </summary>
        private static bool TryEvaluateResidual(PolarErrorDetermination determination,
                                                TopocentricCoordinates observedTopocentric,
                                                RefractionParameters refractionParameters,
                                                DateTime observationTime,
                                                double azimuthErrorDegrees,
                                                double altitudeErrorDegrees,
                                                out double residualAzimuthDegrees,
                                                out double residualAltitudeDegrees) {
            residualAzimuthDegrees = double.NaN;
            residualAltitudeDegrees = double.NaN;

            var predictedTopocentric = GetTopocentricCoordinateForResidual(determination.InitialReferenceFrame.Coordinates,
                                                                           determination.InitialMountAxisAzimuthError.Degree,
                                                                           determination.InitialMountAxisAltitudeError.Degree,
                                                                           azimuthErrorDegrees,
                                                                           altitudeErrorDegrees,
                                                                           determination.Latitude,
                                                                           determination.Longitude,
                                                                           determination.Elevation,
                                                                           refractionParameters,
                                                                           observationTime);

            var meanAltitude = 0.5d * (predictedTopocentric.Altitude.Radians + observedTopocentric.Altitude.Radians);
            residualAzimuthDegrees = NormalizeDegrees(predictedTopocentric.Azimuth.Degree - observedTopocentric.Azimuth.Degree) * Math.Cos(meanAltitude);
            residualAltitudeDegrees = predictedTopocentric.Altitude.Degree - observedTopocentric.Altitude.Degree;

            return double.IsFinite(residualAzimuthDegrees) && double.IsFinite(residualAltitudeDegrees);
        }

        // Normalize an angle difference into the shortest signed branch so the solver always works on the
        // local residual, not on a wrapped 0/360 discontinuity.
        private static double NormalizeDegrees(double degrees) {
            while (degrees > 180d) {
                degrees -= 360d;
            }

            while (degrees < -180d) {
                degrees += 360d;
            }

            return degrees;
        }

        // Closed-form solve for the 2x2 linear system. Since the state is only [az, alt], this is simpler
        // and more transparent than pulling in a general-purpose matrix factorization.
        private static bool TrySolve2x2(double a00, double a01, double a11, double b0, double b1, out double x0, out double x1) {
            var determinant = a00 * a11 - a01 * a01;
            if (Math.Abs(determinant) <= MinimumEigenvalue || !double.IsFinite(determinant)) {
                x0 = 0;
                x1 = 0;
                return false;
            }

            x0 = (a11 * b0 - a01 * b1) / determinant;
            x1 = (a00 * b1 - a01 * b0) / determinant;
            return double.IsFinite(x0) && double.IsFinite(x1);
        }
    }
}
