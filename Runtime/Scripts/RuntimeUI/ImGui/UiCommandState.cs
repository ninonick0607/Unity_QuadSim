// Per-drone UI command state. The UI OWNS these values and streams them each
// rendered frame while UI holds authority (FlightCommandProxy expects a stream —
// stale-command fallback otherwise). Axis4 layouts per SimCore.Common.CommonStructs:
//   Position: (X_m, Y_m, Yaw_deg,     Alt_m)
//   Velocity: (Vx,  Vy,  YawRate_dps, Vz)
//   Angle:    (Roll,Pitch,YawRate_dps,Throttle01)   <- W is THROTTLE, not Z-vel
//   Rate:     (RR,  PR,  YawRate_dps, Throttle01)
using SimCore.Common;

namespace QuadSim.RuntimeUI.ImGuiHud
{
    public sealed class UiCommandState
    {
        // Position mode (output-frame world; Z semantics follow FLU/FRD selection)
        public float PosX, PosY, PosAlt, PosYawDeg;

        // Velocity mode
        public float VelX, VelY, VelZ;

        // Shared across Velocity/Angle/Rate
        public float YawRateDps;
        public float Throttle01;

        // Angle mode
        public float RollDeg, PitchDeg;

        // Rate mode
        public float RollRateDps, PitchRateDps;

        public Axis4 Build(GoalMode mode) => mode switch
        {
            GoalMode.Position => new Axis4(PosX, PosY, PosYawDeg, PosAlt),
            GoalMode.Velocity => new Axis4(VelX, VelY, YawRateDps, VelZ),
            GoalMode.Angle    => new Axis4(RollDeg, PitchDeg, YawRateDps, Throttle01),
            GoalMode.Rate     => new Axis4(RollRateDps, PitchRateDps, YawRateDps, Throttle01),
            _                 => Axis4.ZeroValue,
        };

        public void ZeroAll()
        {
            PosX = PosY = PosAlt = PosYawDeg = 0f;
            VelX = VelY = VelZ = 0f;
            YawRateDps = Throttle01 = 0f;
            RollDeg = PitchDeg = 0f;
            RollRateDps = PitchRateDps = 0f;
        }
    }
}