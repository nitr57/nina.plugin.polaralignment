using FluentAssertions;
using NINA.Core.Model;
using NINA.Core.Model.Equipment;
using NINA.Core.Utility.WindowService;
using NINA.Equipment.Interfaces.Mediator;
using NINA.Image.Interfaces;
using NINA.PlateSolving;
using NINA.PlateSolving.Interfaces;
using NINA.Plugin.Interfaces;
using NINA.Profile.Interfaces;
using NUnit.Framework;
using System.Reflection;

namespace NINA.Plugins.PolarAlignment.Test {
    [TestFixture]
    public class PolarAlignmentSolveCancellationTest {
        [Test]
        public async Task Solve_TreatsSolverCleanupFailureAsCancellationWhenTokenWasCancelled() {
            using var cts = new CancellationTokenSource();
            var rawImageData = Stub.For<IImageData>();
            var renderedImage = Stub.For<IRenderedImage>(handlers => {
                handlers[nameof(IRenderedImage.RawImageData)] = _ => rawImageData;
            });

            var imageSolver = Stub.For<IImageSolver>(handlers => {
                handlers[nameof(IImageSolver.Solve)] = _ => {
                    cts.Cancel();
                    return Task.FromException<PlateSolveResult>(new ArgumentNullException("path1"));
                };
            });
            var plateSolver = Stub.For<IPlateSolver>();
            var plateSolverFactory = Stub.For<IPlateSolverFactory>(handlers => {
                handlers[nameof(IPlateSolverFactory.GetPlateSolver)] = _ => plateSolver;
                handlers[nameof(IPlateSolverFactory.GetImageSolver)] = _ => imageSolver;
            });
            var imagingMediator = Stub.For<IImagingMediator>(handlers => {
                handlers[nameof(IImagingMediator.CaptureAndPrepareImage)] = _ => Task.FromResult(renderedImage);
            });

            var polarAlignment = CreatePolarAlignment(imagingMediator, plateSolverFactory);
            var solve = typeof(Instructions.PolarAlignment).GetMethod("Solve", BindingFlags.Instance | BindingFlags.NonPublic)!;
            var task = (Task<PlateSolveResult>)solve.Invoke(polarAlignment,
                                                            new object[] {
                                                                polarAlignment.TPAPAVM,
                                                                0d,
                                                                new Progress<ApplicationStatus>(),
                                                                cts.Token
                                                            })!;

            var act = async () => await task;

            await act.Should().ThrowAsync<OperationCanceledException>();
        }

        private static Instructions.PolarAlignment CreatePolarAlignment(IImagingMediator imagingMediator, IPlateSolverFactory plateSolverFactory) {
            var astrometrySettings = Stub.For<IAstrometrySettings>(handlers => {
                handlers[nameof(IAstrometrySettings.Latitude)] = _ => 1d;
                handlers[nameof(IAstrometrySettings.Longitude)] = _ => 1d;
                handlers[nameof(IAstrometrySettings.Elevation)] = _ => 0d;
            });
            var cameraSettings = Stub.For<ICameraSettings>(handlers => {
                handlers[nameof(ICameraSettings.PixelSize)] = _ => 3.76d;
            });
            var telescopeSettings = Stub.For<ITelescopeSettings>(handlers => {
                handlers[nameof(ITelescopeSettings.FocalLength)] = _ => 500d;
            });
            var plateSolveSettings = Stub.For<IPlateSolveSettings>(handlers => {
                handlers[nameof(IPlateSolveSettings.Binning)] = _ => (short)1;
                handlers[nameof(IPlateSolveSettings.ExposureTime)] = _ => 1d;
                handlers[nameof(IPlateSolveSettings.DownSampleFactor)] = _ => 1;
                handlers[nameof(IPlateSolveSettings.MaxObjects)] = _ => 500;
            });
            var profile = Stub.For<IProfile>(handlers => {
                handlers[nameof(IProfile.AstrometrySettings)] = _ => astrometrySettings;
                handlers[nameof(IProfile.CameraSettings)] = _ => cameraSettings;
                handlers[nameof(IProfile.TelescopeSettings)] = _ => telescopeSettings;
                handlers[nameof(IProfile.PlateSolveSettings)] = _ => plateSolveSettings;
            });
            var profileService = Stub.For<IProfileService>(handlers => {
                handlers[nameof(IProfileService.ActiveProfile)] = _ => profile;
            });

            var polarAlignment = new Instructions.PolarAlignment(profileService,
                                                                  Stub.For<ICameraMediator>(),
                                                                  imagingMediator,
                                                                  Stub.For<IFilterWheelMediator>(),
                                                                  Stub.For<ITelescopeMediator>(),
                                                                  plateSolverFactory,
                                                                  Stub.For<IDomeMediator>(),
                                                                  Stub.For<IWeatherDataMediator>(),
                                                                  Stub.For<IWindowService>(),
                                                                  Stub.For<IMessageBroker>(),
                                                                  Stub.For<IGuiderMediator>()) {
                Binning = new BinningMode(1, 1),
                SearchRadius = 1
            };

            return polarAlignment;
        }

        private static class Stub {
            public static T For<T>(Action<Dictionary<string, Func<object?[], object?>>>? configure = null) where T : class {
                return StubProxy<T>.For(configure);
            }
        }

        private class StubProxy<T> : DispatchProxy where T : class {
            public Dictionary<string, Func<object?[], object?>> Handlers { get; } = new Dictionary<string, Func<object?[], object?>>();

            public static T For(Action<Dictionary<string, Func<object?[], object?>>>? configure = null) {
                var proxy = Create<T, StubProxy<T>>();
                configure?.Invoke(((StubProxy<T>)(object)proxy).Handlers);
                return proxy;
            }

            protected override object? Invoke(MethodInfo? targetMethod, object?[]? args) {
                if (targetMethod == null) {
                    return null;
                }

                var key = targetMethod.Name;
                if (key.StartsWith("get_", StringComparison.Ordinal)) {
                    key = key.Substring(4);
                }

                if (Handlers.TryGetValue(key, out var handler)) {
                    return handler(args ?? Array.Empty<object?>());
                }

                return DefaultValue(targetMethod.ReturnType);
            }

            private static object? DefaultValue(Type type) {
                if (type == typeof(void)) {
                    return null;
                }

                if (type == typeof(string)) {
                    return string.Empty;
                }

                if (type == typeof(Task)) {
                    return Task.CompletedTask;
                }

                if (type.IsGenericType && type.GetGenericTypeDefinition() == typeof(Task<>)) {
                    var resultType = type.GetGenericArguments()[0];
                    var fromResult = typeof(Task)
                        .GetMethods()
                        .Single(method => method.Name == nameof(Task.FromResult) && method.IsGenericMethod)
                        .MakeGenericMethod(resultType);
                    return fromResult.Invoke(null, new[] { DefaultValue(resultType) });
                }

                if (type.IsValueType) {
                    return Activator.CreateInstance(type);
                }

                if (type.IsInterface) {
                    var factory = typeof(StubProxy<>).MakeGenericType(type).GetMethod(nameof(For), BindingFlags.Public | BindingFlags.Static)!;
                    return factory.Invoke(null, new object?[] { null });
                }

                return null;
            }
        }
    }
}
