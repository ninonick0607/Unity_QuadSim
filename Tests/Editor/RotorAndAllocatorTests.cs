using System.Linq;
using DroneCore.Controls;
using MathUtil;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Yaml.Drone;

namespace Nino.QuadSim.Tests
{
    public sealed class RotorAndAllocatorTests
    {
        [Test]
        public void RotorEquationsMatchGoldenValues()
        {
            const float rho = 1.225f;
            const float n = 28000f / 60f;
            const float diameter = 0.127f;

            Assert.That(RotorMath.StaticThrust(0.11f, rho, n, diameter),
                Is.EqualTo(7.634089f).Within(1e-5f));
            Assert.That(RotorMath.StaticYawTorque(
                    0.040164f, rho, n, diameter, RotorTorqueConvention.Legacy2Point5Pi),
                Is.EqualTo(0.04507288f).Within(1e-7f));
            Assert.That(RotorMath.StaticYawTorque(
                    0.040164f, rho, n, diameter, RotorTorqueConvention.PowerCoefficient2Pi),
                Is.EqualTo(0.05634110f).Within(1e-7f));
        }

        [Test]
        public void MotorLagUsesSeparateExactUpAndDownResponses()
        {
            float up = RotorMath.FirstOrderStep(0f, 1f, 0.004f, 0.0125f, 0.025f);
            float down = RotorMath.FirstOrderStep(1f, 0f, 0.004f, 0.0125f, 0.025f);

            Assert.That(up, Is.EqualTo(0.27385096f).Within(1e-6f));
            Assert.That(down, Is.EqualTo(0.85214379f).Within(1e-6f));
        }

        [Test]
        public void ActuationUsesRotorCrossForceAndSpinTorque()
        {
            RotorInstance[] rotors = QuadX();
            var physics = new RotorPhysicsDerived { MaxThrust = 10f, MaxTorque = 0.2f };
            float[] motors = { 1f, 0f, 0f, 0f };

            QuadActuationModel.ComputeBodyWrench(
                motors, rotors, physics, out Vector3 force, out Vector3 torque);

            Assert.That(force, Is.EqualTo(new Vector3(0f, 10f, 0f)));
            Assert.That(torque.x, Is.EqualTo(-2f).Within(1e-6f));
            Assert.That(torque.y, Is.EqualTo(-0.2f).Within(1e-6f));
            Assert.That(torque.z, Is.EqualTo(2f).Within(1e-6f));
        }

        [Test]
        public void BodyWrenchTransformsToWorld()
        {
            QuadActuationModel.BodyToWorldWrench(
                Quaternion.Euler(0f, 90f, 0f), Vector3.right, Vector3.up,
                out Vector3 forceWorld, out Vector3 torqueWorld);

            Assert.That(forceWorld.x, Is.EqualTo(0f).Within(1e-6f));
            Assert.That(forceWorld.z, Is.EqualTo(-1f).Within(1e-6f));
            Assert.That(torqueWorld, Is.EqualTo(Vector3.up));
        }

        [Test]
        public void SymmetricQuadRoundTripsFeasibleDemand()
        {
            ControlAllocator allocator = BuildAllocator(QuadX());
            float[] motors = new float[4];

            allocator.AllocateYawLimited(0.05f, -0.04f, 0.03f, 0.5f, 0f, motors);

            Assert.That(allocator.Rank, Is.EqualTo(4));
            Assert.That(allocator.ConditionNumber, Is.LessThan(float.PositiveInfinity));
            Assert.That(allocator.LastAllocation.Residual, Is.LessThan(1e-4f));
            Assert.That(motors.All(value => value >= 0f && value <= 1f), Is.True);
        }

        [Test]
        public void PlusQuadRoundTripsFeasibleDemand()
        {
            RotorInstance[] rotors =
            {
                new RotorInstance { ID = 0, Location_Body = new Vector3(0.25f, 0f, 0f), Spin_Dir = -1 },
                new RotorInstance { ID = 1, Location_Body = new Vector3(0f, 0f, -0.25f), Spin_Dir = 1 },
                new RotorInstance { ID = 2, Location_Body = new Vector3(-0.25f, 0f, 0f), Spin_Dir = -1 },
                new RotorInstance { ID = 3, Location_Body = new Vector3(0f, 0f, 0.25f), Spin_Dir = 1 },
            };
            ControlAllocator allocator = BuildAllocator(rotors);
            float[] motors = new float[4];

            allocator.AllocateYawLimited(0.04f, 0.03f, -0.02f, 0.45f, 0f, motors);

            Assert.That(allocator.Rank, Is.EqualTo(4));
            Assert.That(allocator.LastAllocation.Residual, Is.LessThan(1e-4f));
        }

        [Test]
        public void ShiftedGeometryDoesNotBleedCollectiveIntoTorque()
        {
            RotorInstance[] rotors = QuadX();
            for (int i = 0; i < rotors.Length; i++)
                rotors[i].Location_Body += new Vector3(0.03f, 0f, -0.02f);
            ControlAllocator allocator = BuildAllocator(rotors);
            float[] motors = new float[4];

            allocator.AllocateYawLimited(0f, 0f, 0f, 0.4f, 0f, motors);

            Assert.That(allocator.Rank, Is.EqualTo(4));
            Assert.That(allocator.LastAllocation.Achieved.x, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(allocator.LastAllocation.Achieved.y, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(allocator.LastAllocation.Achieved.z, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(allocator.LastAllocation.Achieved.w, Is.EqualTo(0.4f).Within(1e-5f));
        }

        [Test]
        public void SaturationPreservesCollectiveAndDesaturatesYawFirst()
        {
            ControlAllocator allocator = BuildAllocator(QuadX());
            float[] motors = new float[4];

            allocator.AllocateYawLimited(0.05f, 0.04f, 1f, 0.5f, 0f, motors);

            Assert.That(allocator.LastAllocation.RollPitchScale, Is.EqualTo(1f).Within(1e-6f));
            Assert.That(allocator.LastAllocation.YawScale, Is.LessThan(1f));
            Assert.That(allocator.LastAllocation.Achieved.w, Is.EqualTo(0.5f).Within(1e-5f));
            Assert.That(allocator.LastAllocation.SaturatedMotors.Any(value => value), Is.True);
        }

        [TestCase("standard_quad")]
        [TestCase("sentinel_quad")]
        [TestCase("crazyflie2_quad")]
        [TestCase("flightmare_quad")]
        public void PackagedQuadProfilesParseAndHaveFullRank(string profile)
        {
            TextAsset yaml = Resources.Load<TextAsset>($"QuadSim/Configs/Drones/{profile}");
            Assert.That(yaml, Is.Not.Null);
            Assert.That(Parse(yaml.text, profile, out RotorModelInstance propulsion), Is.True);

            var physics = new RotorPhysicsDerived();
            physics.ComputeRotor(propulsion.Model, 0.2f);
            var allocator = new ControlAllocator();
            allocator.RebuildEffectiveness(propulsion.Rotors, physics, SimFrame.FLU);
            Assert.That(allocator.Rank, Is.EqualTo(4));
        }

        [Test]
        public void DuplicateRotorIdsAreRejected()
        {
            LogAssert.Expect(LogType.Error,
                "[Drone.YamlParser] Duplicate propeller id 0: duplicate-test");
            Assert.That(Parse(InvalidYaml(0, 0, 2, 3), "duplicate-test", out _), Is.False);
        }

        [Test]
        public void InvalidSpinDirectionIsRejected()
        {
            LogAssert.Expect(LogType.Error,
                "[Drone.YamlParser] Propeller 1 dir must be -1 or 1: spin-test");
            string yaml = InvalidYaml(0, 1, 2, 3).Replace("id: 1, dir: 1", "id: 1, dir: 0");
            Assert.That(Parse(yaml, "spin-test", out _), Is.False);
        }

        private static ControlAllocator BuildAllocator(RotorInstance[] rotors)
        {
            var physics = new RotorPhysicsDerived { MaxThrust = 10f, MaxTorque = 0.2f };
            var allocator = new ControlAllocator();
            allocator.RebuildEffectiveness(rotors, physics, SimFrame.FLU);
            return allocator;
        }

        private static RotorInstance[] QuadX() => new[]
        {
            new RotorInstance { ID = 0, Location_Body = new Vector3(0.2f, 0f, 0.2f), Spin_Dir = -1 },
            new RotorInstance { ID = 1, Location_Body = new Vector3(0.2f, 0f, -0.2f), Spin_Dir = 1 },
            new RotorInstance { ID = 2, Location_Body = new Vector3(-0.2f, 0f, 0.2f), Spin_Dir = 1 },
            new RotorInstance { ID = 3, Location_Body = new Vector3(-0.2f, 0f, -0.2f), Spin_Dir = -1 },
        };

        private static bool Parse(
            string yaml, string label, out RotorModelInstance propulsion)
        {
            return YamlParser.LoadDroneConfigFromString(
                yaml, label, out _, out _, out propulsion, out _, out _,
                out _, out _, out _, out _);
        }

        private static string InvalidYaml(int id0, int id1, int id2, int id3) => $@"
name: test
propulsion:
  prop_diameter: 0.1
  max_rpm: 10000
  thrust_coef: 0.1
  torque_coef: 0.01
  propellers:
    - {{id: {id0}, dir: -1, x: 0.1, y: 0, z: 0.1}}
    - {{id: {id1}, dir: 1, x: 0.1, y: 0, z: -0.1}}
    - {{id: {id2}, dir: 1, x: -0.1, y: 0, z: 0.1}}
    - {{id: {id3}, dir: -1, x: -0.1, y: 0, z: -0.1}}
";
    }
}
