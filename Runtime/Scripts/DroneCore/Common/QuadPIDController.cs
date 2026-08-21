using UnityEngine;

namespace DroneCore.Common
{
    [System.Serializable]
    public sealed class QuadPIDController
    {
        public float Kp = 0f;
        public float Ki = 0f;
        public float Kd = 0f;

        public float MinOutput = -1f;
        public float MaxOutput = 1f;

        public float IntegralLimit = 10f;
        public float DecayFactor = 0.99f;

        private float _integralSum = 0f;
        private float _prevError = 0f;
        private bool _hasLastError = false;
        private bool _positiveOutputSaturated;
        private bool _negativeOutputSaturated;

        public float IntegralState => _integralSum;

        public void SetGains(float p, float i, float d) { Kp = p; Ki = i; Kd = d; }
        public void SetLimits(float min, float max) { MinOutput = min; MaxOutput = max; }
        //public void SetIntegralLimits(float limit) { IntegralLimit = Mathf.Abs(limit); }
        public void SetDecayFactor(float decay) { DecayFactor = Mathf.Clamp01(decay); }

        public void Reset()
        {
            _integralSum = 0f;
            _prevError = 0f;
            _hasLastError = false;
            ClearSaturation();
        }

        /// <summary>
        /// Feed downstream actuator tracking back into the integrator. This is
        /// evaluated on the next controller update, which matches the one-step
        /// delay between PID calculation and motor allocation.
        /// </summary>
        public void SetSaturationFromTracking(float requested, float achieved, float tolerance = 1e-4f)
        {
            _positiveOutputSaturated = requested > achieved + tolerance;
            _negativeOutputSaturated = requested < achieved - tolerance;
        }

        public void ClearSaturation()
        {
            _positiveOutputSaturated = false;
            _negativeOutputSaturated = false;
        }

        public float Calculate(float desired, float measured, float dt)
        {
            if (dt <= Mathf.Epsilon) return 0f;

            float error = desired - measured;

            // P
            float pTerm = Kp * error;

            // I (leak + conditional integration). Do not retain hidden state
            // while Ki is disabled, and do not integrate farther into a
            // downstream actuator limit reported by the mixer/allocator.
            if (Ki > Mathf.Epsilon)
            {
                _integralSum *= DecayFactor;
                bool pushesFurtherIntoSaturation =
                    (error > 0f && _positiveOutputSaturated) ||
                    (error < 0f && _negativeOutputSaturated);
                if (!pushesFurtherIntoSaturation)
                    _integralSum += error * dt;
                _integralSum = Mathf.Clamp(_integralSum, -IntegralLimit, IntegralLimit);
            }
            else
            {
                _integralSum = 0f;
            }
            float iTerm = Ki * _integralSum;

            // D
            float dTerm = 0f;
            if (_hasLastError)
            {
                float derivative = (error - _prevError) / dt;
                dTerm = Kd * derivative;
            }
            else
            {
                _hasLastError = true;
            }

            _prevError = error;

            // Combine + clamp
            float output = pTerm + iTerm + dTerm;
            output = Mathf.Clamp(output, MinOutput, MaxOutput);

            // Back-calculation anti-windup
            if (Ki > Mathf.Epsilon)
            {
                float maxITerm = MaxOutput - pTerm - dTerm;
                float minITerm = MinOutput - pTerm - dTerm;
                float clampedITerm = Mathf.Clamp(iTerm, minITerm, maxITerm);

                if (!Mathf.Approximately(iTerm, clampedITerm))
                    _integralSum = clampedITerm / Ki;
            }

            return output;
        }
    }
}
