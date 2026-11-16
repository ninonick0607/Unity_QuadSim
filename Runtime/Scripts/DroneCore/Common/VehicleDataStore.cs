using DroneCore.Controllers;
using DroneCore.Controls;
using RobotCore;
using SimCore.Common;
using UnityEngine;

namespace DroneCore.Common
{
    /// <summary>Raw rigid-body state from physics, Unity axes. Privileged: only sensors and the truth source read it.</summary>
    public struct PhysicsState
    {
        public Vector3 PositionUnity;
        public Quaternion RotationUnity;          // body to world
        public Vector3 VelocityUnity;             // world, m/s
        public Vector3 AngularVelocityUnity;      // world, rad/s

        public static PhysicsState FromRigidbody(Rigidbody rb) => new PhysicsState
        {
            PositionUnity = rb.position,
            RotationUnity = rb.rotation,
            VelocityUnity = rb.linearVelocity,
            AngularVelocityUnity = rb.angularVelocity,
        };

        public static PhysicsState Identity => new PhysicsState { RotationUnity = Quaternion.identity };
    }

    /// <summary>What the rotor plant produced this step.</summary>
    public struct ActuatorState
    {
        public Motor4 Thrust01;                // thrust fraction produced, 0..1
        public Motor4 ThrustN;                 // N
        public Motor4 RotorSpeedRadS;          // after the motor lag
        public Motor4 RotorSpeedTargetRadS;    // commanded
        public Vector3 ForceBody;              // net wrench applied, Unity body axes
        public Vector3 TorqueBody;
    }

    /// <summary>What the active controller asked for this step.</summary>
    public struct ControllerStatus
    {
        public CascadedController.CascadeTelemetrySnapshot Targets;
        public AllocationDiagnostics Allocation;
        public int AllocatorRank;
        public float AllocatorConditionNumber;
    }

    public enum VehicleRecord { Truth, Measurements, Estimate, Actuators, Controller }

    /// <summary>
    /// One vehicle's data, shared through a single funnel. Each record has
    /// exactly one writer, which must claim it before publishing; everything
    /// else reads snapshots. Within a physics step the records are written in
    /// a fixed order: Truth, Measurements, Estimate (after physics), then
    /// Controller and Actuators (before the next physics step).
    /// </summary>
    public sealed class VehicleDataStore
    {
        private readonly object[] _writers = new object[5];
        private readonly bool[] _rejectionLogged = new bool[5];

        public PhysicsState Truth { get; private set; } = PhysicsState.Identity;
        public SensorData Measurements { get; private set; }
        public VehicleState Estimate { get; private set; } = new VehicleState { RotationUnity = Quaternion.identity };
        public ActuatorState Actuators { get; private set; }
        public ControllerStatus Controller { get; private set; } = new ControllerStatus { AllocatorConditionNumber = float.PositiveInfinity };

        /// <summary>Become the single writer of a record. Fails if another object already owns it.</summary>
        public bool Claim(VehicleRecord record, object writer)
        {
            object owner = _writers[(int)record];
            if (owner != null && !ReferenceEquals(owner, writer))
            {
                Debug.LogError($"[VehicleDataStore] {record} is already written by {owner}; {writer} cannot claim it.");
                return false;
            }
            _writers[(int)record] = writer;
            return true;
        }

        /// <summary>Give up a record, for example when a controller is replaced.</summary>
        public void Release(VehicleRecord record, object writer)
        {
            if (ReferenceEquals(_writers[(int)record], writer))
                _writers[(int)record] = null;
        }

        public void PublishTruth(object writer, in PhysicsState value)
        {
            if (MayWrite(VehicleRecord.Truth, writer)) Truth = value;
        }

        public void PublishMeasurements(object writer, in SensorData value)
        {
            if (MayWrite(VehicleRecord.Measurements, writer)) Measurements = value;
        }

        public void PublishEstimate(object writer, in VehicleState value)
        {
            if (MayWrite(VehicleRecord.Estimate, writer)) Estimate = value;
        }

        public void PublishActuators(object writer, in ActuatorState value)
        {
            if (MayWrite(VehicleRecord.Actuators, writer)) Actuators = value;
        }

        public void PublishController(object writer, in ControllerStatus value)
        {
            if (MayWrite(VehicleRecord.Controller, writer)) Controller = value;
        }

        private bool MayWrite(VehicleRecord record, object writer)
        {
            if (ReferenceEquals(_writers[(int)record], writer)) return true;
            if (!_rejectionLogged[(int)record])
            {
                Debug.LogError($"[VehicleDataStore] {writer} tried to write {record}, which it has not claimed.");
                _rejectionLogged[(int)record] = true;
            }
            return false;
        }
    }
}
