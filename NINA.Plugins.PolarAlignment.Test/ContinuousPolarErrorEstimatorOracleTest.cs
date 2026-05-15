using FluentAssertions;
using NINA.Astrometry;
using NINA.Core.Utility;
using NINA.PlateSolving;
using System;

namespace NINA.Plugins.PolarAlignment.Test {
    /// <summary>
    /// Oracle-style validation for the continuous correction path.
    ///
    /// The coordinate fixtures in this file were generated outside the production C# code using
    /// Astropy 7.2.0 for the coordinate transforms plus a separate Python implementation of the
    /// Rodrigues rotations in the plugin's local horizontal basis.
    /// That makes these tests a real cross-check against an independent implementation instead of
    /// another exercise of the production math helpers.
    ///
    /// These scenarios validate three things:
    /// 1. The initial three-point solve recovers the oracle-defined initial polar error.
    /// 2. The continuous forward model predicts the same later field center as the oracle.
    /// 3. The nonlinear estimator recovers the oracle-defined residual polar error from that later frame.
    /// </summary>
    public class ContinuousPolarErrorEstimatorOracleTest {
        private sealed class FixedTime : ICustomDateTime {
            private readonly DateTime time;

            public FixedTime(DateTime time) {
                this.time = time;
            }

            public DateTime Now => time;

            public DateTime UtcNow => time;
        }

        public sealed class OracleScenario {
            public string Name { get; init; } = string.Empty;
            public DateTime ReferenceTimeUtc { get; init; }
            public DateTime ObservationTimeUtc { get; init; }
            public double LatitudeDegrees { get; init; }
            public double LongitudeDegrees { get; init; }
            public double ElevationMeters { get; init; }
            public double InitialAzimuthErrorDegrees { get; init; }
            public double InitialAltitudeErrorDegrees { get; init; }
            public double ResidualAzimuthErrorDegrees { get; init; }
            public double ResidualAltitudeErrorDegrees { get; init; }
            public double ForwardToleranceArcSeconds { get; init; }
            public double InverseToleranceArcSeconds { get; init; }
            public CoordinateFixture FirstPoint { get; init; } = null!;
            public CoordinateFixture SecondPoint { get; init; } = null!;
            public CoordinateFixture ThirdPoint { get; init; } = null!;
            public CoordinateFixture CurrentPoint { get; init; } = null!;
        }

        public sealed class CoordinateFixture {
            public CoordinateFixture(double raDegrees, double decDegrees) {
                RightAscensionDegrees = raDegrees;
                DeclinationDegrees = decDegrees;
            }

            public double RightAscensionDegrees { get; }
            public double DeclinationDegrees { get; }
        }

        private static readonly OracleScenario[] OracleScenarios = new[] {
            new OracleScenario() {
                Name = "NorthMidAltitude",
                ReferenceTimeUtc = new DateTime(2024, 10, 1, 21, 0, 0, DateTimeKind.Utc),
                ObservationTimeUtc = new DateTime(2024, 10, 1, 21, 9, 0, DateTimeKind.Utc),
                LatitudeDegrees = 48.0,
                LongitudeDegrees = 7.0,
                ElevationMeters = 250.0,
                InitialAzimuthErrorDegrees = 1.2,
                InitialAltitudeErrorDegrees = -0.7,
                ResidualAzimuthErrorDegrees = 0.18,
                ResidualAltitudeErrorDegrees = -0.11,
                ForwardToleranceArcSeconds = 0.5,
                InverseToleranceArcSeconds = 0.75,
                FirstPoint = new CoordinateFixture(89.45714498853339, 66.65009606193605),
                SecondPoint = new CoordinateFixture(60.63382238906891, 66.85232671761935),
                ThirdPoint = new CoordinateFixture(31.451557865645004, 67.27218543565253),
                CurrentPoint = new CoordinateFixture(30.11892555731704, 67.50218284284956)
            },
            new OracleScenario() {
                Name = "SouthMidAltitude",
                ReferenceTimeUtc = new DateTime(2024, 11, 1, 10, 30, 0, DateTimeKind.Utc),
                ObservationTimeUtc = new DateTime(2024, 11, 1, 10, 44, 0, DateTimeKind.Utc),
                LatitudeDegrees = -33.0,
                LongitudeDegrees = 151.0,
                ElevationMeters = 40.0,
                InitialAzimuthErrorDegrees = -0.9,
                InitialAltitudeErrorDegrees = 0.6,
                ResidualAzimuthErrorDegrees = 0.25,
                ResidualAltitudeErrorDegrees = -0.15,
                ForwardToleranceArcSeconds = 0.5,
                InverseToleranceArcSeconds = 0.5,
                FirstPoint = new CoordinateFixture(243.97781526679935, -61.687489651701185),
                SecondPoint = new CoordinateFixture(274.7047454265593, -62.08206704161046),
                ThirdPoint = new CoordinateFixture(305.69934744008697, -62.219609141188236),
                CurrentPoint = new CoordinateFixture(306.1387133095956, -61.00688840430161)
            },
            new OracleScenario() {
                Name = "WrapAround",
                ReferenceTimeUtc = new DateTime(2024, 7, 4, 3, 0, 0, DateTimeKind.Utc),
                ObservationTimeUtc = new DateTime(2024, 7, 4, 3, 7, 0, DateTimeKind.Utc),
                LatitudeDegrees = 30.0,
                LongitudeDegrees = -17.0,
                ElevationMeters = 2300.0,
                InitialAzimuthErrorDegrees = 0.8,
                InitialAltitudeErrorDegrees = -0.4,
                ResidualAzimuthErrorDegrees = -0.12,
                ResidualAltitudeErrorDegrees = 0.09,
                ForwardToleranceArcSeconds = 0.75,
                InverseToleranceArcSeconds = 0.75,
                FirstPoint = new CoordinateFixture(8.619258501145968, 77.0719144412033),
                SecondPoint = new CoordinateFixture(339.3162761138867, 77.48883207677164),
                ThirdPoint = new CoordinateFixture(308.99729655163503, 77.92996018032848),
                CurrentPoint = new CoordinateFixture(306.01349487291213, 77.37699001231734)
            }
        };

        [TestCaseSource(nameof(OracleScenarios))]
        public void PolarAlignment_OracleScenario_MatchesExternalReference(OracleScenario scenario) {
            var refraction = CreateRefractionParameters();
            var latitude = Angle.ByDegree(scenario.LatitudeDegrees);
            var longitude = Angle.ByDegree(scenario.LongitudeDegrees);

            var firstCoordinates = CreateCoordinates(scenario.FirstPoint, scenario.ReferenceTimeUtc);
            var secondCoordinates = CreateCoordinates(scenario.SecondPoint, scenario.ReferenceTimeUtc);
            var thirdCoordinates = CreateCoordinates(scenario.ThirdPoint, scenario.ReferenceTimeUtc);
            var currentCoordinates = CreateCoordinates(scenario.CurrentPoint, scenario.ObservationTimeUtc);

            var determination = new PolarErrorDetermination(new PlateSolveResult() { Coordinates = thirdCoordinates },
                                                            new Position(firstCoordinates, 0, latitude, longitude, scenario.ElevationMeters, refraction),
                                                            new Position(secondCoordinates, 0, latitude, longitude, scenario.ElevationMeters, refraction),
                                                            new Position(thirdCoordinates, 0, latitude, longitude, scenario.ElevationMeters, refraction),
                                                            latitude,
                                                            longitude,
                                                            scenario.ElevationMeters,
                                                            refraction,
                                                            true);

            determination.InitialMountAxisAzimuthError.Degree.Should().BeApproximately(scenario.InitialAzimuthErrorDegrees,
                                                                                       1.0 / 3600.0,
                                                                                       $"{scenario.Name} initial azimuth error should match the oracle");
            determination.InitialMountAxisAltitudeError.Degree.Should().BeApproximately(scenario.InitialAltitudeErrorDegrees,
                                                                                        1.0 / 3600.0,
                                                                                        $"{scenario.Name} initial altitude error should match the oracle");

            var predictedCurrent = ContinuousPolarErrorEstimator.GetCoordinateForResidual(determination.InitialReferenceFrame.Coordinates,
                                                                                          determination.InitialMountAxisAzimuthError.Degree,
                                                                                          determination.InitialMountAxisAltitudeError.Degree,
                                                                                          scenario.ResidualAzimuthErrorDegrees,
                                                                                          scenario.ResidualAltitudeErrorDegrees,
                                                                                          latitude,
                                                                                          longitude,
                                                                                          scenario.ElevationMeters,
                                                                                          refraction,
                                                                                          scenario.ObservationTimeUtc);

            AngularSeparationArcSeconds(predictedCurrent, currentCoordinates, latitude, longitude, scenario.ElevationMeters, refraction, scenario.ObservationTimeUtc)
                .Should().BeLessThan(scenario.ForwardToleranceArcSeconds, $"{scenario.Name} forward model should match the external oracle");

            var result = ContinuousPolarErrorEstimator.Estimate(determination,
                                                                new PlateSolveResult() { Coordinates = currentCoordinates },
                                                                refraction,
                                                                scenario.InitialAzimuthErrorDegrees * -0.5,
                                                                scenario.InitialAltitudeErrorDegrees * 0.5);

            result.Success.Should().BeTrue($"{scenario.Name} should remain observable under the oracle geometry");
            result.AzimuthErrorDegrees.Should().BeApproximately(scenario.ResidualAzimuthErrorDegrees,
                                                                scenario.InverseToleranceArcSeconds / 3600.0,
                                                                $"{scenario.Name} residual azimuth error should match the Astropy oracle");
            result.AltitudeErrorDegrees.Should().BeApproximately(scenario.ResidualAltitudeErrorDegrees,
                                                                 scenario.InverseToleranceArcSeconds / 3600.0,
                                                                 $"{scenario.Name} residual altitude error should match the Astropy oracle");
            result.ResidualArcSeconds.Should().BeLessThan(scenario.InverseToleranceArcSeconds);
        }

        private static Coordinates CreateCoordinates(CoordinateFixture fixture, DateTime timeUtc) {
            return new Coordinates(Angle.ByDegree(fixture.RightAscensionDegrees),
                                   Angle.ByDegree(fixture.DeclinationDegrees),
                                   Epoch.J2000,
                                   new FixedTime(timeUtc));
        }

        private static RefractionParameters CreateRefractionParameters() {
            return RefractionParameters.GetRefractionParameters(new Equipment.Equipment.MyWeatherData.WeatherDataInfo() {
                Connected = true,
                Pressure = 1005,
                Temperature = 7,
                Humidity = 0.8
            }, 0.574);
        }

        private static double AngularSeparationArcSeconds(Coordinates first,
                                                          Coordinates second,
                                                          Angle latitude,
                                                          Angle longitude,
                                                          double elevation,
                                                          RefractionParameters refractionParameters,
                                                          DateTime observationTime) {
            var firstTopocentric = first.Transform(latitude,
                                                   longitude,
                                                   elevation,
                                                   refractionParameters.PressureHPa,
                                                   refractionParameters.Temperature,
                                                   refractionParameters.RelativeHumidity,
                                                   refractionParameters.Wavelength,
                                                   observationTime);
            var secondTopocentric = second.Transform(latitude,
                                                     longitude,
                                                     elevation,
                                                     refractionParameters.PressureHPa,
                                                     refractionParameters.Temperature,
                                                     refractionParameters.RelativeHumidity,
                                                     refractionParameters.Wavelength,
                                                     observationTime);

            var firstVector = Vector3.CoordinatesToUnitVector(firstTopocentric);
            var secondVector = Vector3.CoordinatesToUnitVector(secondTopocentric);
            var dot = firstVector.X * secondVector.X + firstVector.Y * secondVector.Y + firstVector.Z * secondVector.Z;
            dot = Math.Max(-1d, Math.Min(1d, dot));
            return Math.Acos(dot) * 180d / Math.PI * 3600d;
        }
    }
}
