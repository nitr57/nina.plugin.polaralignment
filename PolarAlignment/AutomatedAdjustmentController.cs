using System;
using System.Collections.Generic;

namespace NINA.Plugins.PolarAlignment {
    /// <summary>
    /// Learns a local linear actuator model from observed error changes and uses that model
    /// to choose bounded correction moves.
    ///
    /// The local model is:
    /// <c>delta_error ~= A * command</c>
    /// where the error vector is <c>[azimuth, altitude]</c> in degrees and the command vector
    /// is <c>[X, Y]</c> in the logical nudge units exposed by the selected hardware system.
    /// </summary>
    internal sealed class AutomatedAdjustmentController {
        /// <summary>
        /// Probe moves are intentionally small and conservative. They exist to identify
        /// the local actuator response, not to make rapid progress.
        /// </summary>
        private const double DefaultProbeMagnitude = 1.0;
        /// <summary>
        /// Commands below this magnitude are ignored to avoid chattering around zero and to
        /// prevent learning from motions that are likely smaller than backlash or slop.
        /// </summary>
        private const double MinimumMoveMagnitude = 0.05;
        /// <summary>
        /// Absolute emergency ceiling on any single move, regardless of what the learned
        /// response model predicts. This exists purely as a backstop against a badly wrong or
        /// still-unconverged model (e.g. a near-singular fit early on) proposing an absurd
        /// command — it is intentionally far above any move that should occur in normal
        /// operation.
        ///
        /// The REAL, proportional limiting factor is the 0.5/0.25/0.125 damping already applied
        /// to the raw least-squares solution in <see cref="CreateCorrectivePlan"/>: since that
        /// raw solution already predicts (given an accurate model) a command that fully zeroes
        /// the current residual, a candidate at 50% of it is inherently proportional to the
        /// current error — large errors produce large (but damped) corrections, small errors
        /// produce small ones. A low fixed ceiling here (previously 5.0, ≈5 arcmin at gear ratio
        /// 1) defeated that proportionality and forced many more solve/move cycles than
        /// necessary for large initial residuals, even though hardware like OAT can safely move
        /// several degrees in one command.
        /// </summary>
        private const double MaximumMoveMagnitude = 240.0; // ≈4° at gear ratio 1.0 — emergency backstop only
        /// <summary>
        /// Small damping term used as a numerical floor and as regularization when
        /// inverting the local response model.
        /// </summary>
        private const double NormalEquationDamping = 1e-6;
        /// <summary>
        /// A candidate move must predict at least a slight reduction in total error before
        /// the controller will accept it.
        /// </summary>
        private const double MinimumExpectedImprovementFactor = 0.99;
        /// <summary>
        /// If a non-probe move makes the measured total error materially worse, the learned
        /// model is treated as stale and discarded.
        /// </summary>
        private const double ModelResetWorseningFactor = 1.05;
        /// <summary>
        /// Maximum number of recent identification samples retained in the local model.
        /// </summary>
        private const int MaxSamples = 12;

        /// <summary>
        /// Below this measured total-error change (in degrees), a commanded move that was
        /// reported as electrically/protocol-successful is treated as having produced no
        /// detectable physical motion. This is deliberately small — well under any sane
        /// single-axis move at any reasonable gear ratio — so it only fires on genuine
        /// non-movement, not on a merely small correction.
        /// </summary>
        private const double StallMovementThresholdDegrees = 0.0015; // ≈0.1 arcmin

        /// <summary>
        /// Number of consecutive "commanded a real move but measured essentially zero change"
        /// events required before declaring a stall. Requiring several in a row avoids false
        /// positives from a single noisy/failed plate solve.
        /// </summary>
        private const int MaxConsecutiveStalls = 3;

        private readonly Queue<ResponseSample> samples = new Queue<ResponseSample>();
        private AutomatedAdjustmentObservation currentObservation;
        private PendingPlan pendingPlan;
        private bool hasObservation;
        private int consecutiveStalls;

        public int SampleCount => samples.Count;

        /// <summary>
        /// True once several consecutive commanded moves have produced no measurable change in
        /// the observed polar error — the physical signature of a motor stuck at an end-stop (or
        /// otherwise mechanically unable to move), even though the driver/INDI layer reported the
        /// command as having completed successfully. Callers should stop issuing further moves
        /// when this is set.
        /// </summary>
        public bool StallDetected { get; private set; }

        /// <summary>Human-readable explanation of the last detected stall, for logging/notification.</summary>
        public string StallReason { get; private set; }

        /// <summary>
        /// Gets whether the current sample set is rich enough and well-conditioned enough
        /// to estimate a two-axis local response model.
        /// </summary>
        public bool HasResponseModel {
            get => TryBuildResponseModel(out _);
        }

        /// <summary>
        /// Clears all learned actuator response state and any pending move bookkeeping.
        /// </summary>
        public void Reset() {
            samples.Clear();
            currentObservation = null;
            pendingPlan = null;
            hasObservation = false;
            consecutiveStalls = 0;
            StallDetected = false;
            StallReason = null;
        }

        /// <summary>
        /// Feeds the latest measured residual error into the controller.
        ///
        /// If a move was executed in the previous cycle, this method converts the before/after
        /// difference into one identification sample of the local actuator response.
        /// </summary>
        public void UpdateObservation(double azimuthErrorDegrees, double altitudeErrorDegrees) {
            var latestObservation = new AutomatedAdjustmentObservation(azimuthErrorDegrees, altitudeErrorDegrees);

            if (pendingPlan != null) {
                var deltaAzimuth = latestObservation.AzimuthErrorDegrees - pendingPlan.BeforeMoveObservation.AzimuthErrorDegrees;
                var deltaAltitude = latestObservation.AltitudeErrorDegrees - pendingPlan.BeforeMoveObservation.AltitudeErrorDegrees;
                AddSample(new ResponseSample(pendingPlan.Plan.XMagnitude,
                                             pendingPlan.Plan.YMagnitude,
                                             deltaAzimuth,
                                             deltaAltitude));

                CheckForStall(pendingPlan.Plan, Math.Sqrt(deltaAzimuth * deltaAzimuth + deltaAltitude * deltaAltitude));

                if (!pendingPlan.Plan.IsProbe && latestObservation.TotalErrorDegrees > pendingPlan.BeforeMoveObservation.TotalErrorDegrees * ModelResetWorseningFactor) {
                    samples.Clear();
                }

                pendingPlan = null;
            }

            currentObservation = latestObservation;
            hasObservation = true;
        }

        /// <summary>
        /// Creates the next automated move.
        ///
        /// Before the response matrix becomes observable, this returns small probe moves
        /// to learn the hardware sign and gain. After that, it returns the safest corrective
        /// move that is predicted to reduce the residual error norm.
        /// </summary>
        public AutomatedAdjustmentPlan CreatePlan() {
            if (StallDetected) {
                return AutomatedAdjustmentPlan.Skip(StallReason ?? "Mount stall detected — automated adjustments paused.");
            }

            if (!hasObservation) {
                return AutomatedAdjustmentPlan.Skip("No continuous error measurement is available yet.");
            }

            if (TryBuildResponseModel(out var responseModel)) {
                var correctivePlan = CreateCorrectivePlan(responseModel, currentObservation);
                if (correctivePlan.HasMovement) {
                    return correctivePlan;
                }
            }

            return CreateProbePlan();
        }

        /// <summary>
        /// Records that a move was executed successfully. The controller waits for the next
        /// measured solve result before turning that move into a training sample.
        /// </summary>
        public void NoteSuccessfulExecution(AutomatedAdjustmentPlan plan) {
            if (!hasObservation || !plan.HasMovement) {
                return;
            }

            pendingPlan = new PendingPlan(plan, currentObservation);
        }

        /// <summary>
        /// Records that the last commanded move failed. Failed moves must not contribute
        /// to the learned actuator model.
        /// </summary>
        public void NoteFailedExecution() {
            pendingPlan = null;
        }

        /// <summary>
        /// Compares a commanded move against the measured error change it produced. A real move
        /// (above the deadband) that yields essentially zero measured change, repeated several
        /// times in a row, is the observable signature of a motor that physically cannot move
        /// (end-stop reached, mechanical jam, disconnected coupler, etc.) — the driver can report
        /// the command as "done" without the mount having actually moved.
        /// </summary>
        private void CheckForStall(AutomatedAdjustmentPlan commandedPlan, double measuredChangeDegrees) {
            var commandedMagnitude = Math.Sqrt(commandedPlan.XMagnitude * commandedPlan.XMagnitude
                                               + commandedPlan.YMagnitude * commandedPlan.YMagnitude);

            if (commandedMagnitude < MinimumMoveMagnitude) {
                // Command was itself negligible — not a meaningful test of whether the mount moves.
                return;
            }

            if (measuredChangeDegrees < StallMovementThresholdDegrees) {
                consecutiveStalls++;
                if (consecutiveStalls >= MaxConsecutiveStalls) {
                    StallDetected = true;
                    StallReason = $"Commanded {commandedMagnitude:F2} units but measured error changed by only " +
                                  $"{measuredChangeDegrees * 60.0:F3}' over {consecutiveStalls} consecutive moves — " +
                                  "the mount does not appear to be responding (possible end-stop or mechanical stall).";
                }
            } else {
                consecutiveStalls = 0;
            }
        }

        private void AddSample(ResponseSample sample) {
            samples.Enqueue(sample);
            while (samples.Count > MaxSamples) {
                samples.Dequeue();
            }
        }

        /// <summary>
        /// Creates a probe move on the less-observed axis so the sample set becomes informative
        /// in both columns of the local response matrix.
        /// </summary>
        private AutomatedAdjustmentPlan CreateProbePlan() {
            var xExcitation = 0.0;
            var yExcitation = 0.0;

            foreach (var sample in samples) {
                xExcitation += Math.Abs(sample.XMagnitude);
                yExcitation += Math.Abs(sample.YMagnitude);
            }

            if (xExcitation <= yExcitation) {
                return new AutomatedAdjustmentPlan(DefaultProbeMagnitude,
                                                   0,
                                                   true,
                                                   "Probing azimuth response");
            }

            return new AutomatedAdjustmentPlan(0,
                                               DefaultProbeMagnitude,
                                               true,
                                               "Probing altitude response");
        }

        /// <summary>
        /// Builds the best available corrective command from the learned response model.
        ///
        /// The controller evaluates a damped two-axis least-squares step and one-axis fallback
        /// moves, then keeps the candidate that predicts the largest reduction in residual norm.
        /// </summary>
        private AutomatedAdjustmentPlan CreateCorrectivePlan(ResponseModel responseModel, AutomatedAdjustmentObservation observation) {
            var currentNorm = observation.TotalErrorDegrees;
            var candidates = new List<AutomatedAdjustmentPlan>();

            if (TrySolveLeastSquaresCommand(responseModel, observation, out var rawX, out var rawY)) {
                candidates.Add(CreateScaledPlan(rawX, rawY, 0.5, "Adaptive two-axis correction"));
                candidates.Add(CreateScaledPlan(rawX, rawY, 0.25, "Adaptive two-axis correction"));
                candidates.Add(CreateScaledPlan(rawX, rawY, 0.125, "Adaptive two-axis correction"));
            }

            if (TryCreateSingleAxisPlan(responseModel.AzimuthDeltaPerXUnit,
                                        responseModel.AltitudeDeltaPerXUnit,
                                        observation,
                                        true,
                                        out var xAxisPlan)) {
                candidates.Add(xAxisPlan);
            }

            if (TryCreateSingleAxisPlan(responseModel.AzimuthDeltaPerYUnit,
                                        responseModel.AltitudeDeltaPerYUnit,
                                        observation,
                                        false,
                                        out var yAxisPlan)) {
                candidates.Add(yAxisPlan);
            }

            AutomatedAdjustmentPlan bestPlan = null;
            var bestPredictedNorm = currentNorm;

            foreach (var candidate in candidates) {
                if (!candidate.HasMovement) {
                    continue;
                }

                var predictedNorm = PredictErrorNorm(responseModel, observation, candidate);
                if (predictedNorm < bestPredictedNorm * MinimumExpectedImprovementFactor) {
                    bestPredictedNorm = predictedNorm;
                    bestPlan = candidate;
                }
            }

            return bestPlan ?? AutomatedAdjustmentPlan.Skip("The learned automation model does not yet predict a safe improvement.");
        }

        /// <summary>
        /// Predicts the post-move residual norm using the current local response model.
        /// </summary>
        private static double PredictErrorNorm(ResponseModel responseModel, AutomatedAdjustmentObservation observation, AutomatedAdjustmentPlan plan) {
            var predictedAzimuth = observation.AzimuthErrorDegrees
                                   + responseModel.AzimuthDeltaPerXUnit * plan.XMagnitude
                                   + responseModel.AzimuthDeltaPerYUnit * plan.YMagnitude;
            var predictedAltitude = observation.AltitudeErrorDegrees
                                    + responseModel.AltitudeDeltaPerXUnit * plan.XMagnitude
                                    + responseModel.AltitudeDeltaPerYUnit * plan.YMagnitude;
            return Math.Sqrt(predictedAzimuth * predictedAzimuth + predictedAltitude * predictedAltitude);
        }

        /// <summary>
        /// Scales and clamps a raw move candidate to the controller's safe operating bounds.
        /// </summary>
        private static AutomatedAdjustmentPlan CreateScaledPlan(double xMagnitude, double yMagnitude, double scale, string reason) {
            return new AutomatedAdjustmentPlan(NormalizeMagnitude(xMagnitude * scale),
                                               NormalizeMagnitude(yMagnitude * scale),
                                               false,
                                               reason);
        }

        /// <summary>
        /// Creates a one-axis fallback move by projecting the current error onto a single
        /// actuator response vector.
        /// </summary>
        private static bool TryCreateSingleAxisPlan(double azimuthDeltaPerUnit,
                                                    double altitudeDeltaPerUnit,
                                                    AutomatedAdjustmentObservation observation,
                                                    bool xAxis,
                                                    out AutomatedAdjustmentPlan plan) {
            var leverage = azimuthDeltaPerUnit * azimuthDeltaPerUnit + altitudeDeltaPerUnit * altitudeDeltaPerUnit;
            if (leverage <= NormalEquationDamping) {
                plan = null;
                return false;
            }

            var command = -((azimuthDeltaPerUnit * observation.AzimuthErrorDegrees)
                           + (altitudeDeltaPerUnit * observation.AltitudeErrorDegrees)) / leverage;
            command = NormalizeMagnitude(command * 0.5);

            if (Math.Abs(command) < MinimumMoveMagnitude) {
                plan = null;
                return false;
            }

            plan = xAxis
                ? new AutomatedAdjustmentPlan(command, 0, false, "Adaptive azimuth correction")
                : new AutomatedAdjustmentPlan(0, command, false, "Adaptive altitude correction");
            return true;
        }

        /// <summary>
        /// Solves the damped least-squares command
        /// <c>min || e + A u ||^2</c>
        /// by forming the corresponding <c>(A^T A + lambda I) u = -A^T e</c> normal equations.
        /// </summary>
        private static bool TrySolveLeastSquaresCommand(ResponseModel responseModel,
                                                        AutomatedAdjustmentObservation observation,
                                                        out double xMagnitude,
                                                        out double yMagnitude) {
            var m00 = responseModel.AzimuthDeltaPerXUnit * responseModel.AzimuthDeltaPerXUnit
                      + responseModel.AltitudeDeltaPerXUnit * responseModel.AltitudeDeltaPerXUnit
                      + NormalEquationDamping;
            var m01 = responseModel.AzimuthDeltaPerXUnit * responseModel.AzimuthDeltaPerYUnit
                      + responseModel.AltitudeDeltaPerXUnit * responseModel.AltitudeDeltaPerYUnit;
            var m11 = responseModel.AzimuthDeltaPerYUnit * responseModel.AzimuthDeltaPerYUnit
                      + responseModel.AltitudeDeltaPerYUnit * responseModel.AltitudeDeltaPerYUnit
                      + NormalEquationDamping;

            var rhs0 = -(responseModel.AzimuthDeltaPerXUnit * observation.AzimuthErrorDegrees
                         + responseModel.AltitudeDeltaPerXUnit * observation.AltitudeErrorDegrees);
            var rhs1 = -(responseModel.AzimuthDeltaPerYUnit * observation.AzimuthErrorDegrees
                         + responseModel.AltitudeDeltaPerYUnit * observation.AltitudeErrorDegrees);

            var determinant = m00 * m11 - m01 * m01;
            if (Math.Abs(determinant) <= NormalEquationDamping) {
                xMagnitude = 0;
                yMagnitude = 0;
                return false;
            }

            xMagnitude = ((rhs0 * m11) - (rhs1 * m01)) / determinant;
            yMagnitude = ((m00 * rhs1) - (m01 * rhs0)) / determinant;
            return true;
        }

        /// <summary>
        /// Fits the local response matrix from the recent sample window using least squares.
        ///
        /// The fit is rejected if the sample geometry is too sparse or too ill-conditioned
        /// to support a reliable two-axis estimate.
        /// </summary>
        private bool TryBuildResponseModel(out ResponseModel responseModel) {
            responseModel = null;

            if (samples.Count < 2) {
                return false;
            }

            var s00 = 0.0;
            var s01 = 0.0;
            var s11 = 0.0;
            var azimuthB0 = 0.0;
            var azimuthB1 = 0.0;
            var altitudeB0 = 0.0;
            var altitudeB1 = 0.0;

            foreach (var sample in samples) {
                s00 += sample.XMagnitude * sample.XMagnitude;
                s01 += sample.XMagnitude * sample.YMagnitude;
                s11 += sample.YMagnitude * sample.YMagnitude;

                azimuthB0 += sample.XMagnitude * sample.AzimuthDeltaDegrees;
                azimuthB1 += sample.YMagnitude * sample.AzimuthDeltaDegrees;
                altitudeB0 += sample.XMagnitude * sample.AltitudeDeltaDegrees;
                altitudeB1 += sample.YMagnitude * sample.AltitudeDeltaDegrees;
            }

            var determinant = s00 * s11 - s01 * s01;
            if (determinant <= NormalEquationDamping) {
                return false;
            }

            // Reject nearly singular sample sets. This is the identification-side equivalent of
            // saying "we have not yet probed the hardware in enough independent directions."
            var trace = s00 + s11;
            var discriminant = Math.Sqrt(Math.Max(0, trace * trace - 4 * determinant));
            var largestEigenvalue = (trace + discriminant) / 2.0;
            var smallestEigenvalue = (trace - discriminant) / 2.0;

            if (smallestEigenvalue <= NormalEquationDamping || largestEigenvalue / smallestEigenvalue > 1e6) {
                return false;
            }

            var inverseS00 = s11 / determinant;
            var inverseS01 = -s01 / determinant;
            var inverseS11 = s00 / determinant;

            responseModel = new ResponseModel(
                azimuthDeltaPerXUnit: inverseS00 * azimuthB0 + inverseS01 * azimuthB1,
                azimuthDeltaPerYUnit: inverseS01 * azimuthB0 + inverseS11 * azimuthB1,
                altitudeDeltaPerXUnit: inverseS00 * altitudeB0 + inverseS01 * altitudeB1,
                altitudeDeltaPerYUnit: inverseS01 * altitudeB0 + inverseS11 * altitudeB1);
            return true;
        }

        /// <summary>
        /// Applies the controller's deadband and move clamp to a raw command magnitude.
        /// </summary>
        private static double NormalizeMagnitude(double magnitude) {
            if (Math.Abs(magnitude) < MinimumMoveMagnitude) {
                return 0;
            }

            if (magnitude > MaximumMoveMagnitude) {
                return MaximumMoveMagnitude;
            }

            if (magnitude < -MaximumMoveMagnitude) {
                return -MaximumMoveMagnitude;
            }

            return magnitude;
        }

        private sealed class PendingPlan {
            public PendingPlan(AutomatedAdjustmentPlan plan, AutomatedAdjustmentObservation beforeMoveObservation) {
                Plan = plan;
                BeforeMoveObservation = beforeMoveObservation;
            }

            public AutomatedAdjustmentPlan Plan { get; }
            public AutomatedAdjustmentObservation BeforeMoveObservation { get; }
        }

        private sealed class ResponseSample {
            public ResponseSample(double xMagnitude, double yMagnitude, double azimuthDeltaDegrees, double altitudeDeltaDegrees) {
                XMagnitude = xMagnitude;
                YMagnitude = yMagnitude;
                AzimuthDeltaDegrees = azimuthDeltaDegrees;
                AltitudeDeltaDegrees = altitudeDeltaDegrees;
            }

            public double XMagnitude { get; }
            public double YMagnitude { get; }
            public double AzimuthDeltaDegrees { get; }
            public double AltitudeDeltaDegrees { get; }
        }

        private sealed class ResponseModel {
            public ResponseModel(double azimuthDeltaPerXUnit,
                                 double azimuthDeltaPerYUnit,
                                 double altitudeDeltaPerXUnit,
                                 double altitudeDeltaPerYUnit) {
                AzimuthDeltaPerXUnit = azimuthDeltaPerXUnit;
                AzimuthDeltaPerYUnit = azimuthDeltaPerYUnit;
                AltitudeDeltaPerXUnit = altitudeDeltaPerXUnit;
                AltitudeDeltaPerYUnit = altitudeDeltaPerYUnit;
            }

            public double AzimuthDeltaPerXUnit { get; }
            public double AzimuthDeltaPerYUnit { get; }
            public double AltitudeDeltaPerXUnit { get; }
            public double AltitudeDeltaPerYUnit { get; }
        }
    }

    internal sealed class AutomatedAdjustmentPlan {
        public AutomatedAdjustmentPlan(double xMagnitude, double yMagnitude, bool isProbe, string reason) {
            XMagnitude = xMagnitude;
            YMagnitude = yMagnitude;
            IsProbe = isProbe;
            Reason = reason;
        }

        public double XMagnitude { get; }
        public double YMagnitude { get; }
        public bool IsProbe { get; }
        public string Reason { get; }
        public bool HasMovement => Math.Abs(XMagnitude) > 0 || Math.Abs(YMagnitude) > 0;

        public static AutomatedAdjustmentPlan Skip(string reason) => new AutomatedAdjustmentPlan(0, 0, false, reason);
    }

    internal sealed class AutomatedAdjustmentObservation {
        public AutomatedAdjustmentObservation(double azimuthErrorDegrees, double altitudeErrorDegrees) {
            AzimuthErrorDegrees = azimuthErrorDegrees;
            AltitudeErrorDegrees = altitudeErrorDegrees;
        }

        public double AzimuthErrorDegrees { get; }
        public double AltitudeErrorDegrees { get; }
        public double TotalErrorDegrees => Math.Sqrt(AzimuthErrorDegrees * AzimuthErrorDegrees + AltitudeErrorDegrees * AltitudeErrorDegrees);
    }
}
