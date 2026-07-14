using NINA.Core.Utility;
using NINA.Profile.Interfaces;

namespace NINA.Plugins.PolarAlignment.OAT {

    /// <summary>
    /// ViewModel for the OpenAstroExplorer / LX200 OAT polar alignment system.
    ///
    /// Inherits the full connect/disconnect, nudge, and automated-adjustment loop
    /// from <see cref="UniversalPolarAlignmentBaseVM"/>.  The only OAT-specific
    /// additions are:
    ///   • <see cref="OATDeviceName"/> — configurable INDI device name
    ///   • <see cref="XSpeed"/> / <see cref="YSpeed"/> are no-ops (OAT ignores speed)
    ///   • No backlash on Y axis (altitude motor has no backlash compensation in the driver)
    /// </summary>
    public class OATAlignmentVM : UniversalPolarAlignmentBaseVM {

        public OATAlignmentVM(IProfileService profileService) : base(profileService) { }

        protected override string SystemName => "OAT Polar Alignment";

        protected override IPolarAlignmentSystem CreateSystem() {
            var name = string.IsNullOrWhiteSpace(Properties.Settings.Default.OATDeviceName)
                ? OATAlignmentDevice.DefaultDeviceName
                : Properties.Settings.Default.OATDeviceName;
            return new OATAlignment(name);
        }

        // ── OAT-specific settings ────────────────────────────────────────────

        public string OATDeviceName {
            get => string.IsNullOrWhiteSpace(Properties.Settings.Default.OATDeviceName)
                ? OATAlignmentDevice.DefaultDeviceName
                : Properties.Settings.Default.OATDeviceName;
            set {
                Properties.Settings.Default.OATDeviceName = value;
                CoreUtil.SaveSettings(Properties.Settings.Default);
                RaisePropertyChanged();
            }
        }

        // ── UniversalPolarAlignmentBaseVM abstract members ───────────────────

        public override bool DoAutomatedAdjustments {
            get => Properties.Settings.Default.OATDoAutomatedAdjustments;
            set {
                Properties.Settings.Default.OATDoAutomatedAdjustments = value;
                CoreUtil.SaveSettings(Properties.Settings.Default);
                RaisePropertyChanged();
            }
        }

        public override double AutomatedAdjustmentSettleTime {
            get => Properties.Settings.Default.OATSettleTime;
            set {
                Properties.Settings.Default.OATSettleTime = value;
                CoreUtil.SaveSettings(Properties.Settings.Default);
                RaisePropertyChanged();
            }
        }

        /// <summary>
        /// AZ gear ratio: 1 logical unit = XGearRatio arcminutes sent to POLAR_AZ.
        /// Default 1.0 is appropriate for direct arcminute control.
        /// </summary>
        public override float XGearRatio {
            get => Properties.Settings.Default.OATXGearRatio;
            set {
                if (value < 0.01f) value = 0.01f;
                Properties.Settings.Default.OATXGearRatio = value;
                if (upa != null) upa.XGearRatio = value;
                CoreUtil.SaveSettings(Properties.Settings.Default);
                RaisePropertyChanged();
                RaisePropertyChanged(nameof(PositionX));
            }
        }

        /// <summary>Speed is ignored by the OAT driver; exposed for interface compatibility.</summary>
        public override int XSpeed {
            get => 0;
            set { /* OAT manages its own speed */ }
        }

        /// <summary>
        /// ALT gear ratio: 1 logical unit = YGearRatio arcminutes sent to POLAR_ALT.
        /// Default 1.0.
        /// </summary>
        public override float YGearRatio {
            get => Properties.Settings.Default.OATYGearRatio;
            set {
                if (value < 0.01f) value = 0.01f;
                Properties.Settings.Default.OATYGearRatio = value;
                if (upa != null) upa.YGearRatio = value;
                CoreUtil.SaveSettings(Properties.Settings.Default);
                RaisePropertyChanged();
                RaisePropertyChanged(nameof(PositionY));
            }
        }

        public override int YSpeed {
            get => 0;
            set { /* OAT manages its own speed */ }
        }

        public override bool ReverseAzimuth {
            get => Properties.Settings.Default.OATReverseAzimuth;
            set {
                Properties.Settings.Default.OATReverseAzimuth = value;
                CoreUtil.SaveSettings(Properties.Settings.Default);
                RaisePropertyChanged();
            }
        }

        public override bool ReverseAltitude {
            get => Properties.Settings.Default.OATReverseAltitude;
            set {
                Properties.Settings.Default.OATReverseAltitude = value;
                CoreUtil.SaveSettings(Properties.Settings.Default);
                RaisePropertyChanged();
            }
        }

        /// <summary>Backlash compensation on the AZ axis (arcmin).</summary>
        public override float XBacklashCompensation {
            get => Properties.Settings.Default.OATXBacklashCompensation;
            set {
                Properties.Settings.Default.OATXBacklashCompensation = value;
                CoreUtil.SaveSettings(Properties.Settings.Default);
                RaisePropertyChanged();
            }
        }
    }
}
