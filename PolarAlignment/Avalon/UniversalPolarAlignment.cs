using System.Text.RegularExpressions;

namespace NINA.Plugins.PolarAlignment.Avalon {
    public partial class UniversalPolarAlignment : UniversalPolarAlignmentBase {
        protected override string SystemName => "Avalon Polar Alignment System";

        private float xGearRatio = Properties.Settings.Default.AvalonXGearRatio;
        private float yGearRatio = Properties.Settings.Default.AvalonYGearRatio;

        public override float XGearRatio { get => xGearRatio; set => xGearRatio = value; }
        public override float YGearRatio { get => yGearRatio; set => yGearRatio = value; }

        protected override Regex GetStatusRegex() => StatusRegex();

        [GeneratedRegex(@"<(?<status>\w+)\|MPos:(?<x>[+-]?\d+(\.\d+)?),(?<y>[+-]?\d+(\.\d+)?),(?<z>[+-]?\d+(\.\d+)?)\|")]
        private static partial Regex StatusRegex();
    }
}
