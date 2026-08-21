namespace SimCore.Common
{
    public static class FlightAxes
    {
        public const int X   = 0;
        public const int Y   = 1;
        public const int Yaw = 2;
        public const int Z   = 3;
    }
    public static class FlightCommandLayout
    {
        // Position mode expects: (X_m, Y_m, Yaw_deg_or_nan, Z_m)
        public static Axis4 Position(float x_m, float y_m, float z_m, float yaw_deg)
            => new Axis4(x_m, y_m, yaw_deg, z_m);

        // Velocity mode expects: (Vx_mps, Vy_mps, YawRate_deg_s, Vz_mps)
        public static Axis4 Velocity(float vx_mps, float vy_mps, float vz_mps, float yawRate_deg_s)
            => new Axis4(vx_mps, vy_mps, yawRate_deg_s, vz_mps);

        // Angle mode: (Roll_deg, Pitch_deg, YawRate_deg_s, Throttle01)
        public static Axis4 Angle(float roll_deg, float pitch_deg, float yawRate_deg_s, float throttle01)
            => new Axis4(roll_deg, pitch_deg, yawRate_deg_s, throttle01);

        // Rate mode: (RollRate_deg_s, PitchRate_deg_s, YawRate_deg_s, Throttle01)
        public static Axis4 Rate(float rollRate_deg_s, float pitchRate_deg_s, float yawRate_deg_s, float throttle01)
            => new Axis4(rollRate_deg_s, pitchRate_deg_s, yawRate_deg_s, throttle01);

        public static Axis4 Wrench(float tx, float ty, float tz, float fz)
            => new Axis4(tx, ty, tz, fz);

        // yaw lives at index 2 for pos/vel (your UE comment: "yaw lives in Z" meaning the 3rd slot)
        public static bool HasYaw(in Axis4 cmd) => IsFinite(cmd[FlightAxes.Yaw]);

        // .NET 5+ has float.IsFinite; this works everywhere.
        private static bool IsFinite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
    }
}
