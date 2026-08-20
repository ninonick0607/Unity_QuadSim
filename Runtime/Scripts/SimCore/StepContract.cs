using System;

namespace SimCore
{
    public enum StepTargetStatus
    {
        Accepted,
        Missing,
        LateOrDuplicate,
        CountMismatch,
        Skipped,
    }

    public static class StepContract
    {
        public static StepTargetStatus Validate(
            long currentStep, int count, bool strict, bool hasTarget,
            long targetStep, out long expectedStep)
        {
            expectedStep = currentStep + Math.Max(1, count);
            if (strict && !hasTarget) return StepTargetStatus.Missing;
            if (!hasTarget || targetStep == expectedStep) return StepTargetStatus.Accepted;
            if (targetStep <= currentStep) return StepTargetStatus.LateOrDuplicate;
            if (targetStep < expectedStep) return StepTargetStatus.CountMismatch;
            return StepTargetStatus.Skipped;
        }
    }
}
