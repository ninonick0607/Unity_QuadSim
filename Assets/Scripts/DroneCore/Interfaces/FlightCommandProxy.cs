using SimCore.Common;
using UnityEngine;

namespace DroneCore.Interfaces
{
    public sealed class FlightCommandProxy : MonoBehaviour, ICommandSource
    {
        [SerializeField] private Axis4 _command;
        [SerializeField] private GoalMode _mode = GoalMode.None;
        [SerializeField] private InputSource _lastWriter = InputSource.UI;
        [SerializeField] private InputSource _activeSource = InputSource.UI;

        public Axis4 GetCommandValue() => _command;
        public GoalMode GetGoalMode() => _mode;
        public InputSource GetActiveSource() => _activeSource;
        public InputSource GetLastWriter() => _lastWriter;

        public void SetActiveSource(InputSource inActive)
        {
            if (_activeSource == inActive) return;

            Debug.Log($"[Proxy] {GetInstanceID():X} ActiveSource: {_activeSource} -> {inActive} (LastWriter={_lastWriter})");
            _activeSource = inActive;
        }

        public bool SetCommand(Axis4 inCmd, GoalMode inMode, InputSource caller)
        {
            if (caller != _activeSource)
            {
                Debug.LogWarning($"[Proxy] REJECT {GetInstanceID():X} Caller={caller} Active={_activeSource} Mode={inMode} Cmd=({inCmd.X:F2} {inCmd.Y:F2} {inCmd.Z:F2} {inCmd.W:F2})");
                return false;
            }

            _command = inCmd;
            _mode = inMode;
            _lastWriter = caller;
            return true;
        }

        public void ForceSetCommand(Axis4 inCmd, GoalMode inMode, InputSource caller)
        {
            _command = inCmd;
            _mode = inMode;
            _lastWriter = caller;
        }
    }
}