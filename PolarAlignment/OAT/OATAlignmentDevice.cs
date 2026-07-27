using NINA.Core.Utility;
using NINA.INDI;
using NINA.INDI.Devices;
using NINA.INDI.Enums;
using NINA.INDI.Protocol;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

namespace NINA.Plugins.PolarAlignment.OAT {

    /// <summary>
    /// Lightweight proxy that registers with the already-running LX200 OAT INDI driver
    /// and forwards polar alignment motor commands via the POLAR_ALT / POLAR_AZ INDI
    /// number properties, without touching the telescope CONNECTION switch.
    ///
    /// INDI property mapping (from lx200_OpenAstroTech.cpp):
    ///   POLAR_ALT  /  element OAT_POLAR_ALT  — relative altitude move in arcminutes
    ///                 underlying command: :MAL{value}#   range: -140..140 arcmin
    ///   POLAR_AZ   /  element OAT_POLAR_AZ   — relative azimuth move in arcminutes
    ///                 underlying command: :MAZ{value}#   range: -320..320 arcmin
    ///
    /// Completion is signalled by the driver broadcasting state=Ok after state=Busy on
    /// the corresponding property (some OAT firmware versions skip Busy and go directly
    /// to Ok).  We use polling rather than a TCS callback to avoid a race between
    /// stale GetProperties responses and the actual command response.
    /// </summary>
    internal sealed class OATAlignmentDevice : INDIDevice {

        public const string DefaultDeviceName = "LX200 OpenAstroTech";

        // Maximum time to wait for a move to complete (Arduino may be slow)
        private static readonly TimeSpan DefaultMoveTimeout = TimeSpan.FromSeconds(90);

        // Set to the UTC time just before we call SendProperty; used in callbacks
        // to distinguish stale GetProperties echoes from command responses.
        // Note: 'volatile' is not permitted on 'long' fields in C# (CS0677) — atomicity
        // for writes is already provided by Interlocked.Exchange at every write site.
        private long _azCommandSentAtTicks  = 0;
        private long _altCommandSentAtTicks = 0;

        // Completion sources for in-flight moves; null when no move is pending.
        private volatile TaskCompletionSource<bool> _altMoveCompletion;
        private volatile TaskCompletionSource<bool> _azMoveCompletion;

        public OATAlignmentDevice(string deviceName)
            : base(new INDIDeviceInfo {
                Id     = deviceName,
                Name   = deviceName,
                Driver = "indi_lx200_OpenAstroTech"
            }) {
        }

        // ── Public state ────────────────────────────────────────────────────

        public bool IsAltMoving => GetNumberProperty("POLAR_ALT")?.State == PropertyState.Busy;
        public bool IsAzMoving  => GetNumberProperty("POLAR_AZ")?.State  == PropertyState.Busy;
        public bool IsMoving    => IsAltMoving || IsAzMoving;

        public string AlignmentStatus {
            get {
                if (IsMoving) return "Moving";
                var altProp = GetNumberProperty("POLAR_ALT");
                if (altProp == null) return "Properties not yet received";
                return altProp.State == PropertyState.Alert ? "Alert" : "Idle";
            }
        }

        // ── Move commands ────────────────────────────────────────────────────

        /// <summary>
        /// Sends a relative altitude move of <paramref name="arcmin"/> arcminutes and
        /// returns a Task that completes when the driver reports the move is finished.
        /// </summary>
        public async Task SendMoveAlt(double arcmin, CancellationToken token) {
            Logger.Info($"[OATAlignment] SendMoveAlt {arcmin:F3} arcmin");

            // Note state BEFORE sending so we can detect when state changes
            var stateBefore = GetNumberProperty("POLAR_ALT")?.State;
            Logger.Debug($"[OATAlignment] POLAR_ALT state before command: {stateBefore}");

            // Create TCS and stamp the send time BEFORE registering the callback window
            var tcs = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            var reg = token.Register(() => tcs.TrySetCanceled(), useSynchronizationContext: false);
            tcs.Task.ContinueWith(_ => reg.Dispose(), TaskContinuationOptions.ExecuteSynchronously);

            Interlocked.Exchange(ref _altCommandSentAtTicks, DateTime.UtcNow.Ticks);
            _altMoveCompletion = tcs;

            INDIClient.Instance.SendProperty(BuildNumberProperty("POLAR_ALT", "OAT_POLAR_ALT", arcmin));
            Logger.Debug("[OATAlignment] POLAR_ALT setNumber sent, awaiting completion callback...");

            try {
                using var timeoutCts = CancellationTokenSource.CreateLinkedTokenSource(token);
                timeoutCts.CancelAfter(DefaultMoveTimeout);
                var timeoutReg = timeoutCts.Token.Register(
                    () => tcs.TrySetException(new TimeoutException("POLAR_ALT move timed out")),
                    useSynchronizationContext: false);
                try   { await tcs.Task; }
                finally { timeoutReg.Dispose(); }
            } catch {
                _altMoveCompletion = null;
                throw;
            }

            Logger.Debug($"[OATAlignment] POLAR_ALT move complete. New state: {GetNumberProperty("POLAR_ALT")?.State}");
        }

        /// <summary>
        /// Sends a relative azimuth move of <paramref name="arcmin"/> arcminutes and
        /// returns a Task that completes when the driver reports the move is finished.
        /// </summary>
        public async Task SendMoveAz(double arcmin, CancellationToken token) {
            Logger.Info($"[OATAlignment] SendMoveAz {arcmin:F3} arcmin");

            // Note state BEFORE sending so we can detect when state changes
            var stateBefore = GetNumberProperty("POLAR_AZ")?.State;
            Logger.Debug($"[OATAlignment] POLAR_AZ state before command: {stateBefore}");

            var tcs = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            var reg = token.Register(() => tcs.TrySetCanceled(), useSynchronizationContext: false);
            tcs.Task.ContinueWith(_ => reg.Dispose(), TaskContinuationOptions.ExecuteSynchronously);

            Interlocked.Exchange(ref _azCommandSentAtTicks, DateTime.UtcNow.Ticks);
            _azMoveCompletion = tcs;

            INDIClient.Instance.SendProperty(BuildNumberProperty("POLAR_AZ", "OAT_POLAR_AZ", arcmin));
            Logger.Debug("[OATAlignment] POLAR_AZ setNumber sent, awaiting completion callback...");

            try {
                using var timeoutCts = CancellationTokenSource.CreateLinkedTokenSource(token);
                timeoutCts.CancelAfter(DefaultMoveTimeout);
                var timeoutReg = timeoutCts.Token.Register(
                    () => tcs.TrySetException(new TimeoutException("POLAR_AZ move timed out")),
                    useSynchronizationContext: false);
                try   { await tcs.Task; }
                finally { timeoutReg.Dispose(); }
            } catch {
                _azMoveCompletion = null;
                throw;
            }

            Logger.Debug($"[OATAlignment] POLAR_AZ move complete. New state: {GetNumberProperty("POLAR_AZ")?.State}");
        }

        // ── INDI property callbacks ──────────────────────────────────────────

        public override void OnNumberPropertyUpdated(INDINumberProperty p) {
            // Log every POLAR_ALT / POLAR_AZ callback regardless of whether a move is pending
            if (p.Name == "POLAR_ALT" || p.Name == "POLAR_AZ") {
                var hasTcs  = p.Name == "POLAR_ALT" ? _altMoveCompletion != null : _azMoveCompletion != null;
                var sentAt  = p.Name == "POLAR_ALT" ? _altCommandSentAtTicks : _azCommandSentAtTicks;
                var lagMs   = sentAt == 0 ? -1.0
                              : (DateTime.UtcNow.Ticks - sentAt) / TimeSpan.TicksPerMillisecond;
                Logger.Debug(
                    $"[OATAlignment] OnNumberPropertyUpdated {p.Name} state={p.State} " +
                    $"hasPendingTCS={hasTcs} lagSinceCommand={lagMs:F0}ms");
            }

            // The driver sets state=Busy when it starts the motor, then Ok (or Alert)
            // when the motor stops.  We only resolve the TCS on terminal states.
            //
            // Guard: ignore callbacks that arrive before we ever sent a command
            // (lagSinceCommand < 0) — these are stale GetProperties echoes.
            if (p.Name == "POLAR_ALT" && p.State != PropertyState.Busy) {
                // Only accept if we already sent a command (commandSentAt != 0)
                if (_altCommandSentAtTicks == 0) {
                    Logger.Debug("[OATAlignment] Ignoring POLAR_ALT callback — no command sent yet (stale GetProperties).");
                    return;
                }
                var tcs = _altMoveCompletion;
                _altMoveCompletion = null;
                if (p.State == PropertyState.Alert)
                    tcs?.TrySetException(new InvalidOperationException("POLAR_ALT driver returned Alert state."));
                else
                    tcs?.TrySetResult(true);
            }
            else if (p.Name == "POLAR_AZ" && p.State != PropertyState.Busy) {
                if (_azCommandSentAtTicks == 0) {
                    Logger.Debug("[OATAlignment] Ignoring POLAR_AZ callback — no command sent yet (stale GetProperties).");
                    return;
                }
                var tcs = _azMoveCompletion;
                _azMoveCompletion = null;
                if (p.State == PropertyState.Alert)
                    tcs?.TrySetException(new InvalidOperationException("POLAR_AZ driver returned Alert state."));
                else
                    tcs?.TrySetResult(true);
            }
        }

        public override void OnTextPropertyUpdated(INDITextProperty p)     { }
        public override void OnSwitchPropertyUpdated(INDISwitchProperty p) { }

        // ── Dispose ──────────────────────────────────────────────────────────

        /// <summary>
        /// Unregisters from INDIClient WITHOUT sending CONNECTION=Off so the telescope
        /// session is not affected.
        /// </summary>
        public new void Dispose() {
            try {
                _altMoveCompletion?.TrySetCanceled();
                _azMoveCompletion?.TrySetCanceled();
                INDIClient.Instance.UnregisterDevice(this);
            } catch (Exception ex) {
                Logger.Error($"[OATAlignment] Dispose error: {ex.Message}");
            }
        }

        // ── Private helpers ──────────────────────────────────────────────────

        private INDINumberProperty BuildNumberProperty(string propName, string elementName, double value) {
            return new INDINumberProperty {
                DeviceName = Id,
                Name       = propName,
                Numbers    = new List<INDINumber> {
                    new INDINumber { Name = elementName, Value = value }
                }
            };
        }
    }
}
