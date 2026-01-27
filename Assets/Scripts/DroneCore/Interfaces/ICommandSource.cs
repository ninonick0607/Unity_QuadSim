using RobotCore.Common;

namespace DroneCore.Interfaces
{
    public interface ICommandSource
    {
        Axis4 Command { get; }
        GoalMode Mode { get; }
        InputSource Source { get; }
    }
}