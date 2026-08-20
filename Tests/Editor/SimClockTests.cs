using NUnit.Framework;
using SimCore;

namespace Nino.QuadSim.Tests
{
    public sealed class SimClockTests
    {
        [Test]
        public void FixedClockTimeIsDerivedFromIntegerStepCount()
        {
            var clock = new SteppableClock(1.0 / 250.0);

            for (int i = 0; i < 100_000; i++) clock.Step();

            Assert.That(clock.StepCount, Is.EqualTo(100_000));
            Assert.That(clock.StepSizeNanos, Is.EqualTo(4_000_000));
            Assert.That(clock.NowNanos, Is.EqualTo(400_000_000_000));
        }

        [Test]
        public void ResetClearsStepAndPreservesRequestedEpoch()
        {
            var clock = new SteppableClock(0.004, 10);
            clock.StepBy(0.020);
            clock.Reset(1_000);

            Assert.That(clock.StepCount, Is.Zero);
            Assert.That(clock.NowNanos, Is.EqualTo(1_000));
            Assert.That(clock.StartNanos, Is.EqualTo(1_000));
        }

        [TestCase(10, 5, true, false, 0, StepTargetStatus.Missing)]
        [TestCase(10, 5, true, true, 10, StepTargetStatus.LateOrDuplicate)]
        [TestCase(10, 5, true, true, 12, StepTargetStatus.CountMismatch)]
        [TestCase(10, 5, true, true, 20, StepTargetStatus.Skipped)]
        [TestCase(10, 5, true, true, 15, StepTargetStatus.Accepted)]
        public void StrictStepContractClassifiesInvalidTargets(
            long current, int count, bool strict, bool hasTarget, long target,
            StepTargetStatus expectedStatus)
        {
            StepTargetStatus status = StepContract.Validate(
                current, count, strict, hasTarget, target, out long expectedStep);

            Assert.That(status, Is.EqualTo(expectedStatus));
            Assert.That(expectedStep, Is.EqualTo(15));
        }
    }
}
