namespace DroneCore.Interfaces
{
    public enum GoalMode : byte
    {
        None,
        Position,   // Cmd = {PosX_m, PosY_m, YawAngle_deg, PosZ_m}
        Velocity,   // Cmd = {VelX_mps, VelY_mps, YawRate_dps, VelZ_mps}
        Angle,      // Cmd = {Roll_deg, Pitch_deg, YawRate_dps, VelZ_mps}
        Rate        // Cmd = {RollRate_dps, PitchRate_dps, YawRate_dps, Throttle_01}
    }

    public enum InputSource : byte
    {
        UI,
        Gamepad,
        API,
        Navigation
    }
}