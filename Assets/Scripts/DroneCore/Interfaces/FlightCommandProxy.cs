using RobotCore.Common;
using UnityEngine;

namespace DroneCore.Interfaces
{
    public readonly struct CommandSnapshot
    {
        public readonly Axis4 Command;
        public readonly GoalMode Mode;
        public readonly InputSource Source;

        public CommandSnapshot(Axis4 cmd, GoalMode mode, InputSource source)
        {
            Command = cmd;
            Mode = mode;
            Source = source;
        }
    }

    /// <summary>
    /// Central command hub. Producers write commands; controllers read stable snapshots.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class FlightCommandProxy : MonoBehaviour, ICommandSource
    {
        [Header("Debug / Default Safe State")]
        [SerializeField] private GoalMode defaultMode = GoalMode.None;
        [SerializeField] private InputSource defaultSource = InputSource.UI;

        // "front" is what sim reads.
        private Axis4 _frontCmd;
        private GoalMode _frontMode;
        private InputSource _frontSource;

        // "back" is what UI writes (optional). If you don't need buffering,
        // you can write directly to front by setting useDoubleBuffer=false.
        private Axis4 _backCmd;
        private GoalMode _backMode;
        private InputSource _backSource;

        [Tooltip("If true, writers update a back-buffer and sim swaps once per step.")]
        [SerializeField] private bool useDoubleBuffer = true;

        public Axis4 Command => _frontCmd;
        public GoalMode Mode => _frontMode;
        public InputSource Source => _frontSource;

<<<<<<< HEAD
=======
        
>>>>>>> 714525f (Lots of additions since last build, trying to match unreals model)
        private void Awake()
        {
            Debug.Log($"[CmdProxy] Awake on {name}");
            ResetToSafe();
        }

        public void SetCommand(Axis4 cmd, GoalMode mode, InputSource source = InputSource.UI)
        {
            Debug.Log($"[ControlDeck] Write Rate cmd rpy=({cmd.Roll:F2},{cmd.Pitch:F2},{cmd.Yaw:F2}) mode={GoalMode.Rate} ");

            if (useDoubleBuffer)
            {
                _backCmd = cmd;
                _backMode = mode;
                _backSource = source;
            }
            else
            {
                _frontCmd = cmd;
                _frontMode = mode;
                _frontSource = source;
            }
        }

        public void ResetToSafe()
        {
            var safe = Axis4.ZeroValue;
            if (useDoubleBuffer)
            {
                _backCmd = safe;
                _backMode = defaultMode;
                _backSource = defaultSource;
            }

            _frontCmd = safe;
            _frontMode = defaultMode;
            _frontSource = defaultSource;
        }

        /// <summary>
        /// Call this exactly once at the start of PrePhysicsStep (or sim tick) to make inputs coherent.
        /// </summary>
        public void LatchForStep()
        {
            if (!useDoubleBuffer) return;

            _frontCmd = _backCmd;
            _frontMode = _backMode;
            _frontSource = _backSource;
        }

        public CommandSnapshot GetSnapshot()
        {
            // Snapshot is a value copy. Safe for the rest of the step.
            return new CommandSnapshot(_frontCmd, _frontMode, _frontSource);
        }
    }
}
