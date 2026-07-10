namespace Experimental
{
    using UnityEngine;

    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(ConfigurableJoint))]
    public sealed class PayloadAttachment : MonoBehaviour
    {
        private Rigidbody _body;
        private ConfigurableJoint _joint;

        private void Awake()
        {
            _body = GetComponent<Rigidbody>();
            _joint = GetComponent<ConfigurableJoint>();
        }

        public void AttachTo(Rigidbody droneBody, Transform mount)
        {
            if (droneBody == null || mount == null)
            {
                Debug.LogError("[PayloadAttachment] Missing drone Rigidbody or mount.");
                return;
            }

            // Position the payload attachment point at the drone mount.
            _joint.connectedBody = null;
            _body.position = mount.position;
            _body.rotation = mount.rotation;

            // Avoid an initial impulse if the drone is already moving.
            _body.linearVelocity = droneBody.GetPointVelocity(mount.position);
            _body.angularVelocity = droneBody.angularVelocity;

            // WaterBottle root origin is the payload-side attachment point.
            _joint.autoConfigureConnectedAnchor = false;
            _joint.anchor = Vector3.zero;

            // Connected anchor is expressed in the drone Rigidbody's local frame.
            _joint.connectedAnchor =
                droneBody.transform.InverseTransformPoint(mount.position);

            _joint.connectedBody = droneBody;
        }
    }
}