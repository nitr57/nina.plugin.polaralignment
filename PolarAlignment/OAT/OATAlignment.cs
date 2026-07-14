using NINA.Core.Utility;
using NINA.INDI;
using System;
using System.Threading;
using System.Threading.Tasks;

namespace NINA.Plugins.PolarAlignment.OAT {

    /// <summary>
    /// <see cref="IPolarAlignmentSystem"/> implementation for the OpenAstroExplorer
    /// mount controlled by the INDI LX200 OpenAstroTech driver.
    ///
    /// Unlike the serial-port-based OAPA / Avalon systems, the OAT's polar alignment
    /// motors are embedded in the telescope INDI driver.  This class routes movement
    /// commands through the already-connected INDIClient rather than opening a new
    /// serial port.
    ///
    /// Axis mapping:
    ///   XAxis  →  Azimuth   (INDI: POLAR_AZ  / element OAT_POLAR_AZ,  arcmin)
    ///   YAxis  →  Altitude  (INDI: POLAR_ALT / element OAT_POLAR_ALT, arcmin)
    ///
    /// Gear ratio:
    ///   Default is 1.0 for both axes (1 logical unit = 1 arcminute).
    ///   The AutomatedAdjustmentController learns the actual response empirically,
    ///   so this only affects the initial probe step size.
    /// </summary>
    public sealed class OATAlignment : IPolarAlignmentSystem {

        // Maximum time to wait for a single motor move before declaring a timeout.
        private static readonly TimeSpan DefaultMoveTimeout = TimeSpan.FromSeconds(120);

        private readonly OATAlignmentDevice _device;

        // Accumulated logical-unit positions (arcmin when gearRatio = 1).
        private float _xPosition = 0f; // AZ
        private float _yPosition = 0f; // ALT

        // ── Construction ─────────────────────────────────────────────────────

        /// <param name="deviceName">
        /// INDI device name as reported by indiserver, e.g. "LX200 OpenAstroTech".
        /// </param>
        /// <exception cref="InvalidOperationException">
        /// Thrown when INDI is not connected or the device is unknown (telescope not connected).
        /// </exception>
        public OATAlignment(string deviceName) {
            if (!INDIClient.Instance.IsConnected)
                throw new InvalidOperationException(
                    "INDI server is not connected. Connect the OAT telescope in NINA first.");

            if (!INDIClient.Instance.IsDeviceKnown(deviceName))
                throw new InvalidOperationException(
                    $"INDI device '{deviceName}' is not connected. " +
                    "Make sure the OAT telescope is connected in NINA.");

            _device = new OATAlignmentDevice(deviceName);
            Logger.Info($"[OATAlignment] Connected to INDI device '{deviceName}'.");
        }

        // ── IPolarAlignmentSystem ────────────────────────────────────────────

        public bool Connected =>
            INDIClient.Instance.IsConnected &&
            INDIClient.Instance.IsDeviceKnown(_device.Id);

        public string Status => _device.AlignmentStatus;

        // Accumulated positions in logical units (arcmin for default gearRatio=1)
        public float XPosition1 => _xPosition / XGearRatio; // AZ
        public float YPosition1 => _yPosition / YGearRatio; // ALT
        public float ZPosition1 => 0f;

        public float XGearRatio { get; set; } = 1f;
        public float YGearRatio { get; set; } = 1f;
        public float ZGearRatio { get; set; } = 1f;

        public LastDirection XLastDirection { get; private set; } = LastDirection.Positive;
        public LastDirection YLastDirection { get; private set; } = LastDirection.Positive;
        public LastDirection ZLastDirection { get; private set; } = LastDirection.Positive;

        /// <summary>
        /// Moves the specified axis by <paramref name="position"/> logical units
        /// (arcmin when gearRatio = 1) and waits until the INDI driver reports
        /// the move is complete.
        /// </summary>
        public async Task MoveRelative(Axis axis, int speed, float position, CancellationToken token) {
            // 'speed' is intentionally ignored — the OAT manages its own motor speed.
            switch (axis) {
                case Axis.XAxis: {
                    XLastDirection = position >= 0 ? LastDirection.Positive : LastDirection.Negative;
                    double arcmin = position * XGearRatio;
                    _xPosition += (float)arcmin;
                    using var timeoutCts = CancellationTokenSource.CreateLinkedTokenSource(token);
                    timeoutCts.CancelAfter(DefaultMoveTimeout);
                    await _device.SendMoveAz(arcmin, timeoutCts.Token);
                    break;
                }
                case Axis.YAxis: {
                    YLastDirection = position >= 0 ? LastDirection.Positive : LastDirection.Negative;
                    double arcmin = position * YGearRatio;
                    _yPosition += (float)arcmin;
                    using var timeoutCts = CancellationTokenSource.CreateLinkedTokenSource(token);
                    timeoutCts.CancelAfter(DefaultMoveTimeout);
                    await _device.SendMoveAlt(arcmin, timeoutCts.Token);
                    break;
                }
                default:
                    throw new ArgumentException($"Axis {axis} is not supported by OAT polar alignment.");
            }
        }

        /// <summary>
        /// Moves the axis to an absolute logical-unit position by computing the delta
        /// from the internally tracked current position.
        /// </summary>
        public Task MoveAbsolute(Axis axis, int speed, float position, CancellationToken token) {
            float currentPos = axis == Axis.XAxis ? XPosition1 : YPosition1;
            float delta = position - currentPos;
            return MoveRelative(axis, speed, delta, token);
        }

        /// <summary>
        /// No-op: property state is refreshed automatically via INDI event callbacks.
        /// </summary>
        public Task RefreshStatus(CancellationToken token) => Task.CompletedTask;

        public void Dispose() {
            _device?.Dispose();
        }
    }
}
