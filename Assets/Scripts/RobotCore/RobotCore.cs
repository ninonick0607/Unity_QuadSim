using UnityEngine;

namespace QuadSim.RobotCore
{
    public sealed class RobotCore : MonoBehaviour
    {
        [SerializeField] private SensorManager sensors;

        public SensorManager Sensors => sensors;

        private void Awake()
        {
            if (sensors == null)
                sensors = GetComponentInChildren<SensorManager>(includeInactive: true);
        }
    }
}