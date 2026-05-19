using FluentAssertions;
using NINA.Astrometry;
using NINA.Core.Utility;
using NINA.PlateSolving;
using System.IO;
using System.Text.Json;

namespace NINA.Plugins.PolarAlignment.Test {
    /// <summary>
    /// Broad oracle sweep for the continuous estimator and the initial three-point solve.
    ///
    /// Unlike the hand-picked oracle cases, this suite walks a dense grid of sky positions generated
    /// externally with Astropy and independent Rodrigues rotations. The goal is not just to confirm
    /// that the method works in a few good cases, but to map where it remains tight and where it
    /// starts to drift.
    /// </summary>
    public class ContinuousPolarErrorEstimatorOracleSweepTest {
        private sealed class FixedTime : ICustomDateTime {
            private readonly DateTime time;

            public FixedTime(DateTime time) {
                this.time = time;
            }

            public DateTime Now => time;

            public DateTime UtcNow => time;
        }

        private sealed class OracleSweepManifest {
            public string Generator { get; init; } = string.Empty;
            public string AstropyVersion { get; init; } = string.Empty;
            public string NumpyVersion { get; init; } = string.Empty;
            public string ScipyVersion { get; init; } = string.Empty;
            public double PressureHPa { get; init; }
            public double TemperatureC { get; init; }
            public double RelativeHumidity { get; init; }
            public double WavelengthMicron { get; init; }
            public double MeasurementRotationDegrees { get; init; }
            public int ScenarioCount { get; init; }
            public OracleSweepScenario[] Scenarios { get; init; } = Array.Empty<OracleSweepScenario>();
        }

        private sealed class OracleSweepScenario {
            public string Name { get; init; } = string.Empty;
            public string SiteName { get; init; } = string.Empty;
            public DateTime ReferenceTimeUtc { get; init; }
            public DateTime ObservationTimeUtc { get; init; }
            public double LatitudeDegrees { get; init; }
            public double LongitudeDegrees { get; init; }
            public double ElevationMeters { get; init; }
            public double InitialAzimuthErrorDegrees { get; init; }
            public double InitialAltitudeErrorDegrees { get; init; }
            public double ResidualAzimuthErrorDegrees { get; init; }
            public double ResidualAltitudeErrorDegrees { get; init; }
            public double FirstAzimuthDegrees { get; init; }
            public double FirstAltitudeDegrees { get; init; }
            public double SecondAzimuthDegrees { get; init; }
            public double SecondAltitudeDegrees { get; init; }
            public double ReferenceAzimuthDegrees { get; init; }
            public double ReferenceAltitudeDegrees { get; init; }
            public double CurrentAzimuthDegrees { get; init; }
            public double CurrentAltitudeDegrees { get; init; }
            public double MinimumScenarioAltitudeDegrees { get; init; }
            public CoordinateFixture FirstPoint { get; init; } = null!;
            public CoordinateFixture SecondPoint { get; init; } = null!;
            public CoordinateFixture ThirdPoint { get; init; } = null!;
            public CoordinateFixture CurrentPoint { get; init; } = null!;
        }

        private sealed class CoordinateFixture {
            public double RightAscensionDegrees { get; init; }
            public double DeclinationDegrees { get; init; }
        }

        private sealed class SweepDiagnostic {
            public required string Name { get; init; }
            public required string SiteName { get; init; }
            public required double CurrentAltitudeDegrees { get; init; }
            public required double CurrentAzimuthDegrees { get; init; }
            public required double MinimumScenarioAltitudeDegrees { get; init; }
            public required double InitialAzimuthErrorArcSeconds { get; init; }
            public required double InitialAltitudeErrorArcSeconds { get; init; }
            public required double ForwardSeparationArcSeconds { get; init; }
            public required double InverseAzimuthErrorArcSeconds { get; init; }
            public required double InverseAltitudeErrorArcSeconds { get; init; }
            public required double InverseResidualArcSeconds { get; init; }
            public required bool Success { get; init; }
        }

        [Test]
        public void PolarAlignment_OracleSweep_ReportsMeasuredAccuracyEnvelope() {
            var manifest = LoadManifest();
            var diagnostics = new List<SweepDiagnostic>();

            foreach (var scenario in manifest.Scenarios) {
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

                var estimate = ContinuousPolarErrorEstimator.Estimate(determination,
                                                                      new PlateSolveResult() { Coordinates = currentCoordinates },
                                                                      refraction,
                                                                      scenario.InitialAzimuthErrorDegrees * -0.5,
                                                                      scenario.InitialAltitudeErrorDegrees * 0.5);

                diagnostics.Add(new SweepDiagnostic() {
                    Name = scenario.Name,
                    SiteName = scenario.SiteName,
                    CurrentAltitudeDegrees = scenario.CurrentAltitudeDegrees,
                    CurrentAzimuthDegrees = scenario.CurrentAzimuthDegrees,
                    MinimumScenarioAltitudeDegrees = scenario.MinimumScenarioAltitudeDegrees,
                    InitialAzimuthErrorArcSeconds = Math.Abs(determination.InitialMountAxisAzimuthError.Degree - scenario.InitialAzimuthErrorDegrees) * 3600.0,
                    InitialAltitudeErrorArcSeconds = Math.Abs(determination.InitialMountAxisAltitudeError.Degree - scenario.InitialAltitudeErrorDegrees) * 3600.0,
                    ForwardSeparationArcSeconds = AngularSeparationArcSeconds(predictedCurrent,
                                                                              currentCoordinates,
                                                                              latitude,
                                                                              longitude,
                                                                              scenario.ElevationMeters,
                                                                              refraction,
                                                                              scenario.ObservationTimeUtc),
                    InverseAzimuthErrorArcSeconds = estimate.Success ? Math.Abs(estimate.AzimuthErrorDegrees - scenario.ResidualAzimuthErrorDegrees) * 3600.0 : double.PositiveInfinity,
                    InverseAltitudeErrorArcSeconds = estimate.Success ? Math.Abs(estimate.AltitudeErrorDegrees - scenario.ResidualAltitudeErrorDegrees) * 3600.0 : double.PositiveInfinity,
                    InverseResidualArcSeconds = estimate.ResidualArcSeconds,
                    Success = estimate.Success
                });
            }

            var operationalSweep = diagnostics.Where(diagnostic => diagnostic.MinimumScenarioAltitudeDegrees >= 5.0).ToArray();
            var operationalNonEastWestSweep = diagnostics.Where(diagnostic => diagnostic.MinimumScenarioAltitudeDegrees >= 5.0 && DistanceToEastWestDegrees(diagnostic.CurrentAzimuthDegrees) > 5.0).ToArray();
            var operationalEastWestSweep = diagnostics.Where(diagnostic => diagnostic.MinimumScenarioAltitudeDegrees >= 5.0 && DistanceToEastWestDegrees(diagnostic.CurrentAzimuthDegrees) <= 5.0).ToArray();
            var lowAltitudeSweep = diagnostics.Where(diagnostic => diagnostic.MinimumScenarioAltitudeDegrees < 5.0).ToArray();
            var nearZenithSweep = diagnostics.Where(diagnostic => diagnostic.CurrentAltitudeDegrees >= 75.0).ToArray();

            foreach (var group in diagnostics.GroupBy(GetAltitudeBand).OrderBy(group => group.Key)) {
                var maxInitial = group.Max(diagnostic => Math.Max(diagnostic.InitialAzimuthErrorArcSeconds, diagnostic.InitialAltitudeErrorArcSeconds));
                var maxForward = group.Max(diagnostic => diagnostic.ForwardSeparationArcSeconds);
                var maxInverseComponent = group.Max(diagnostic => Math.Max(diagnostic.InverseAzimuthErrorArcSeconds, diagnostic.InverseAltitudeErrorArcSeconds));
                var maxResidual = group.Max(diagnostic => diagnostic.InverseResidualArcSeconds);
                var failedCount = group.Count(diagnostic => !diagnostic.Success);

                TestContext.WriteLine($"{group.Key}: count={group.Count()}, maxInitial={maxInitial:F3}\" maxForward={maxForward:F3}\" maxInverseComponent={maxInverseComponent:F3}\" maxResidual={maxResidual:F3}\" failures={failedCount}");
            }

            WriteSummary("operational>=5", diagnostics.Where(diagnostic => diagnostic.MinimumScenarioAltitudeDegrees >= 5.0));
            WriteSummary("operational>=5 not east/west", diagnostics.Where(diagnostic => diagnostic.MinimumScenarioAltitudeDegrees >= 5.0 && DistanceToEastWestDegrees(diagnostic.CurrentAzimuthDegrees) > 5.0));
            WriteSummary("operational>=5 east/west", diagnostics.Where(diagnostic => diagnostic.MinimumScenarioAltitudeDegrees >= 5.0 && DistanceToEastWestDegrees(diagnostic.CurrentAzimuthDegrees) <= 5.0));
            WriteSummary("lowAltitude<5", diagnostics.Where(diagnostic => diagnostic.MinimumScenarioAltitudeDegrees < 5.0));
            WriteSummary("nearZenith>=75", diagnostics.Where(diagnostic => diagnostic.CurrentAltitudeDegrees >= 75.0));

            WriteWorstCases("initial", diagnostics, diagnostic => Math.Max(diagnostic.InitialAzimuthErrorArcSeconds, diagnostic.InitialAltitudeErrorArcSeconds));
            WriteWorstCases("forward", diagnostics, diagnostic => diagnostic.ForwardSeparationArcSeconds);
            WriteWorstCases("inverse", diagnostics, diagnostic => Math.Max(diagnostic.InverseAzimuthErrorArcSeconds, diagnostic.InverseAltitudeErrorArcSeconds));

            operationalSweep.Should().NotBeEmpty("the sweep should include fully operational scenarios with every point at least 5 degrees above the horizon");
            operationalNonEastWestSweep.Should().NotBeEmpty("the sweep should include favorable operational cases away from the known east/west singularity");
            operationalEastWestSweep.Should().NotBeEmpty("the sweep should include exact east/west cases so diagnostics continue to cover that known weak region");
            lowAltitudeSweep.Should().NotBeEmpty("the sweep should include low-altitude cases so diagnostics continue to cover horizon-sensitive behavior");
            nearZenithSweep.Should().NotBeEmpty("the sweep should include high-altitude cases to guard against zenith-adjacent regressions");

            MaxInitialComponentArcSeconds(operationalNonEastWestSweep).Should().BeLessThan(1.0, "favorable operational cases should keep the initial three-point solve within about one arcsecond of the Astropy oracle");
            MaxForwardArcSeconds(operationalNonEastWestSweep).Should().BeLessThan(1.0, "favorable operational cases should keep the continuous forward model within about one arcsecond of the Astropy oracle");
            MaxInverseComponentArcSeconds(operationalNonEastWestSweep).Should().BeLessThan(1.0, "favorable operational cases should keep the residual inversion within about one arcsecond of the Astropy oracle");
            MaxResidualArcSeconds(operationalNonEastWestSweep).Should().BeLessThan(0.01, "favorable operational cases should converge to a negligible internal residual");

            MaxInitialComponentArcSeconds(nearZenithSweep).Should().BeLessThan(1.0, "high-altitude non-singular cases should not degrade the initial solve");
            MaxForwardArcSeconds(nearZenithSweep).Should().BeLessThan(0.5, "high-altitude non-singular cases should keep the forward model tightly aligned with the oracle");
            MaxInverseComponentArcSeconds(nearZenithSweep).Should().BeLessThan(1.0, "high-altitude non-singular cases should keep the inverse solution within about one arcsecond");

            diagnostics.Should().OnlyContain(diagnostic => diagnostic.Success, "the current sweep should stay solvable for the generated oracle scenarios");
        }

        private static string GetAltitudeBand(SweepDiagnostic diagnostic) {
            if (diagnostic.CurrentAltitudeDegrees < 2.0) {
                return "<2";
            }
            if (diagnostic.CurrentAltitudeDegrees < 3.0) {
                return "2-3";
            }
            if (diagnostic.CurrentAltitudeDegrees < 4.0) {
                return "3-4";
            }
            if (diagnostic.CurrentAltitudeDegrees < 5.0) {
                return "4-5";
            }
            return ">=5";
        }

        private static double DistanceToEastWestDegrees(double azimuthDegrees) {
            var distanceToEast = Math.Abs(NormalizeSignedDegrees(azimuthDegrees - 90.0));
            var distanceToWest = Math.Abs(NormalizeSignedDegrees(azimuthDegrees - 270.0));
            return Math.Min(distanceToEast, distanceToWest);
        }

        private static double NormalizeSignedDegrees(double degrees) {
            while (degrees > 180.0) {
                degrees -= 360.0;
            }
            while (degrees < -180.0) {
                degrees += 360.0;
            }
            return degrees;
        }

        private static void WriteSummary(string label, IEnumerable<SweepDiagnostic> diagnostics) {
            var materialized = diagnostics.ToArray();
            if (materialized.Length == 0) {
                return;
            }

            var maxInitial = materialized.Max(diagnostic => Math.Max(diagnostic.InitialAzimuthErrorArcSeconds, diagnostic.InitialAltitudeErrorArcSeconds));
            var maxForward = materialized.Max(diagnostic => diagnostic.ForwardSeparationArcSeconds);
            var maxInverseComponent = materialized.Max(diagnostic => Math.Max(diagnostic.InverseAzimuthErrorArcSeconds, diagnostic.InverseAltitudeErrorArcSeconds));
            var maxResidual = materialized.Max(diagnostic => diagnostic.InverseResidualArcSeconds);
            TestContext.WriteLine($"{label}: count={materialized.Length}, maxInitial={maxInitial:F3}\" maxForward={maxForward:F3}\" maxInverseComponent={maxInverseComponent:F3}\" maxResidual={maxResidual:F3}\"");
        }

        private static void WriteWorstCases(string label, IEnumerable<SweepDiagnostic> diagnostics, Func<SweepDiagnostic, double> selector) {
            foreach (var diagnostic in diagnostics.OrderByDescending(selector).Take(8)) {
                TestContext.WriteLine($"{label} worst: {diagnostic.Name} alt={diagnostic.CurrentAltitudeDegrees:F2} az={diagnostic.CurrentAzimuthDegrees:F2} minAlt={diagnostic.MinimumScenarioAltitudeDegrees:F2} initial={Math.Max(diagnostic.InitialAzimuthErrorArcSeconds, diagnostic.InitialAltitudeErrorArcSeconds):F3}\" forward={diagnostic.ForwardSeparationArcSeconds:F3}\" inverse={Math.Max(diagnostic.InverseAzimuthErrorArcSeconds, diagnostic.InverseAltitudeErrorArcSeconds):F3}\" residual={diagnostic.InverseResidualArcSeconds:F3}\"");
            }
        }

        private static double MaxInitialComponentArcSeconds(IEnumerable<SweepDiagnostic> diagnostics) {
            return diagnostics.Max(diagnostic => Math.Max(diagnostic.InitialAzimuthErrorArcSeconds, diagnostic.InitialAltitudeErrorArcSeconds));
        }

        private static double MaxForwardArcSeconds(IEnumerable<SweepDiagnostic> diagnostics) {
            return diagnostics.Max(diagnostic => diagnostic.ForwardSeparationArcSeconds);
        }

        private static double MaxInverseComponentArcSeconds(IEnumerable<SweepDiagnostic> diagnostics) {
            return diagnostics.Max(diagnostic => Math.Max(diagnostic.InverseAzimuthErrorArcSeconds, diagnostic.InverseAltitudeErrorArcSeconds));
        }

        private static double MaxResidualArcSeconds(IEnumerable<SweepDiagnostic> diagnostics) {
            return diagnostics.Max(diagnostic => diagnostic.InverseResidualArcSeconds);
        }

        private static OracleSweepManifest LoadManifest() {
            var fixturePath = Path.Combine(TestContext.CurrentContext.TestDirectory,
                                           "OracleFixtures",
                                           "continuous_polar_error_estimator_oracle_sweep.json");
            var json = File.ReadAllText(fixturePath);
            var manifest = JsonSerializer.Deserialize<OracleSweepManifest>(json, new JsonSerializerOptions() { PropertyNameCaseInsensitive = true });
            manifest.Should().NotBeNull();
            manifest!.ScenarioCount.Should().Be(manifest.Scenarios.Length);
            return manifest;
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
