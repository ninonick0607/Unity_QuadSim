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

        private float _integralSum = 0f;
        private float _prevMeasured = 0f;
        private bool _hasPrevMeasured = false;
        private bool _positiveOutputSaturated;
        private bool _negativeOutputSaturated;

        public float IntegralState => _integralSum;

        public void SetGains(float p, float i, float d) { Kp = p; Ki = i; Kd = d; }
        public void SetLimits(float min, float max) { MinOutput = min; MaxOutput = max; }
        //public void SetIntegralLimits(float limit) { IntegralLimit = Mathf.Abs(limit); }

        public void Reset()
        {
            _integralSum = 0f;
            _prevMeasured = 0f;
            _hasPrevMeasured = false;
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

            // D on measurement: -Kd * d(measured)/dt. While the setpoint is
            // constant this equals Kd * d(error)/dt, but a setpoint step no
            // longer produces a derivative spike ("derivative kick").
            float dTerm = 0f;
            if (_hasPrevMeasured)
                dTerm = -Kd * (measured - _prevMeasured) / dt;
            _prevMeasured = measured;
            _hasPrevMeasured = true;

            // I with conditional integration as the only anti-windup: do not
            // integrate when the output is already at a limit (this PID's own
            // clamp, or a downstream actuator limit reported through
            // SetSaturationFromTracking) and the error would push further into
            // it. No leak, so a constant disturbance is removed completely and
            // the result does not depend on the controller rate. Do not retain
            // hidden state while Ki is disabled.
            if (Ki > Mathf.Epsilon)
            {
                float unclamped = pTerm + Ki * _integralSum + dTerm;
                bool atUpperLimit = unclamped >= MaxOutput || _positiveOutputSaturated;
                bool atLowerLimit = unclamped <= MinOutput || _negativeOutputSaturated;
                bool pushesFurtherIntoSaturation =
                    (error > 0f && atUpperLimit) ||
                    (error < 0f && atLowerLimit);
                if (!pushesFurtherIntoSaturation)
                    _integralSum += error * dt;
                _integralSum = Mathf.Clamp(_integralSum, -IntegralLimit, IntegralLimit);
            }
            else
            {
                _integralSum = 0f;
            }
            float iTerm = Ki * _integralSum;

            // Combine + clamp
            float output = pTerm + iTerm + dTerm;
            return Mathf.Clamp(output, MinOutput, MaxOutput);
        }
    }
}
