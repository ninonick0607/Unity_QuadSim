using MathUtil;
using RobotCore;
using RobotCore.Sensors;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Common
{
    /// <summary>
    /// Turns a vehicle's measurements (or, when opted in, physics truth) into
    /// the state estimate its controller reads. Sole writer of
    /// VehicleRecord.Estimate. An EKF becomes another source here.
    /// </summary>
    public sealed class StateEstimator
    {
        private readonly VehicleDataStore _store;

        public StateSource Source { get; set; } = StateSource.Sensors;

        public StateEstimator(VehicleDataStore store)
        {
            _store = store;
            _store.Claim(VehicleRecord.Estimate, this);
        }

        /// <summary>Publish the estimate for the latest Truth and Measurements.</summary>
        public void Update()
        {
            SensorData measurements = _store.Measurements;
            VehicleState estimate = (Source == StateSource.Truth)
                ? FromPhysics(_store.Truth, OutputFrame(measurements))
                : FromMeasurements(measurements);
            _store.PublishEstimate(this, estimate);
        }

        /// <summary>
        /// State as the sensors report it: IMU attitude, rates and orientation,
        /// GPS position at its own rate, noise when enabled. Velocity has no
        /// real sensor until an estimator exists; it is the IMU velocity sample
        /// (a stand-in for an estimator output) rotated into the level heading
        /// frame with the IMU orientation.
        /// </summary>
        public static VehicleState FromMeasurements(in SensorData s)
        {
            Quaternion rotation = s.ImuValid ? s.ImuRotationUnity : Quaternion.identity;
            Vector3 velocityWorldUnity = rotation * Frames.InverseTransformLinear(s.ImuVel, OutputFrame(s));
            return new VehicleState
            {
                PositionEnu = s.GpsPosition,
                HeadingVelocity = Frames.EnuToHeadingFlu(
                    Frames.TransformWorldVector(velocityWorldUnity, SimFrame.FLU),
                    Frames.EnuHeadingRad(rotation)),
                AttitudeDeg = s.ImuAttitude,
                BodyRatesRadS = s.ImuAngVel,
                RotationUnity = rotation,
            };
        }

        /// <summary>Exact physics state, in the same units and frames as FromMeasurements.</summary>
        public static VehicleState FromPhysics(in PhysicsState p, SimFrame frame)
        {
            Quaternion rotation = p.RotationUnity;
            return new VehicleState
            {
                PositionEnu = Frames.TransformWorldPosition(p.PositionUnity, SimFrame.FLU),
                HeadingVelocity = Frames.EnuToHeadingFlu(
                    Frames.TransformWorldVector(p.VelocityUnity, SimFrame.FLU),
                    Frames.EnuHeadingRad(rotation)),
                AttitudeDeg = IMUSensor.AttitudeDeg(rotation, frame),
                BodyRatesRadS = Frames.TransformAngularVelocity(
                    Quaternion.Inverse(rotation) * p.AngularVelocityUnity, frame),
                RotationUnity = rotation,
            };
        }

        /// <summary>The body frame the measurements are expressed in (FLU before the first sample).</summary>
        public static SimFrame OutputFrame(in SensorData s) =>
            s.BodyFrame == SimFrame.UnityBody ? SimFrame.FLU : s.BodyFrame;
    }
}
