namespace Experimental
{
    using UnityEngine;

    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(ConfigurableJoint))]
    public sealed class PayloadAttachment : MonoBehaviour
    {
        private Rigidbody _body;
        private ConfigurableJoint _joint;
        [SerializeField] private LineRenderer rope;
        [SerializeField] private Transform ropeEnd;

        private Transform _droneMount;
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
            _droneMount = mount;
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
        
        private void LateUpdate()
        {
            if (rope == null || _droneMount == null || ropeEnd == null)
                return;

            rope.positionCount = 2;
            rope.SetPosition(0, _droneMount.position);
            rope.SetPosition(1, ropeEnd.position);
        }
    }
}