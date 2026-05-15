using FluentAssertions;
using System;

namespace NINA.Plugins.PolarAlignment.Test {
    public class UniversalPolarAlignmentBaseTest {
        [Test]
        public void CalculateMovementTimeout_ScalesWithDistanceAndFeedRate() {
            var timeout = UniversalPolarAlignmentBase.CalculateMovementTimeout(0f, 220f, 700);

            timeout.TotalSeconds.Should().BeApproximately(42.7143, 0.0001);
        }

        [Test]
        public void CalculateMovementTimeout_KeepsShortMovesAboveMinimum() {
            var timeout = UniversalPolarAlignmentBase.CalculateMovementTimeout(0f, 0.001f, 1000);

            timeout.TotalSeconds.Should().BeApproximately(5, 0.0001);
        }

        [Test]
        public void CalculateMovementTimeout_UsesFallbackForInvalidSpeed() {
            var timeout = UniversalPolarAlignmentBase.CalculateMovementTimeout(0f, 10f, 0);

            timeout.TotalSeconds.Should().BeApproximately(30, 0.0001);
        }
    }
}
