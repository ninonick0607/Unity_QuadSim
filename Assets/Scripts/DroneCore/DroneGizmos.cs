using UnityEngine;

namespace QuadSim.DroneCore
{
    public sealed class DroneGizmos : MonoBehaviour
    {
        public float axisLength = 0.25f;

        private void OnDrawGizmos()
        {
            Vector3 origin = transform.position;

            Gizmos.color = Color.red;
            Gizmos.DrawLine(origin, origin + transform.right * axisLength);

            Gizmos.color = Color.green;
            Gizmos.DrawLine(origin, origin + transform.up * axisLength);

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(origin, origin + transform.forward * axisLength);
        }
    }
}