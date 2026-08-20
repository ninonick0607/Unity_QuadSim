using System;

namespace MathUtil
{
    public enum RotorTorqueConvention
    {
        Legacy2Point5Pi = 0,
        PowerCoefficient2Pi = 1,
    }

    public static class RotorMath
    {
        public const float SeaLevelAirDensity = 1.225f;

        public static float StaticThrust(
            float thrustCoefficient, float airDensity, float revolutionsPerSecond,
            float propellerDiameter)
        {
            double n2 = revolutionsPerSecond * revolutionsPerSecond;
            double d4 = Math.Pow(propellerDiameter, 4);
            return (float)(thrustCoefficient * airDensity * n2 * d4);
        }

        public static float StaticYawTorque(
            float torqueCoefficient, float airDensity, float revolutionsPerSecond,
            float propellerDiameter, RotorTorqueConvention convention)
        {
            double denominator = convention == RotorTorqueConvention.PowerCoefficient2Pi
                ? 2.0 * Math.PI
                : 2.5 * Math.PI;
            double n2 = revolutionsPerSecond * revolutionsPerSecond;
            double d5 = Math.Pow(propellerDiameter, 5);
            return (float)(torqueCoefficient * airDensity * n2 * d5 / denominator);
        }

        public static float CommandToThrust(float command01, float maxThrust, float airDensityRatio = 1f)
        {
            return Clamp01(command01) * maxThrust * airDensityRatio;
        }

        public static float CommandToYawTorque(
            float command01, float maxTorque, int turningDirection, float airDensityRatio = 1f)
        {
            return Clamp01(command01) * maxTorque * turningDirection * airDensityRatio;
        }

        public static float FirstOrderStep(
            float current, float target, float dtSeconds,
            float tauUpSeconds, float tauDownSeconds)
        {
            if (dtSeconds <= 0f) return current;
            float tau = target >= current ? tauUpSeconds : tauDownSeconds;
            if (tau <= 0f) return target;
            double alpha = 1.0 - Math.Exp(-dtSeconds / tau);
            return current + (target - current) * (float)alpha;
        }

        private static float Clamp01(float value)
        {
            if (value < 0f) return 0f;
            if (value > 1f) return 1f;
            return value;
        }
    }
}
