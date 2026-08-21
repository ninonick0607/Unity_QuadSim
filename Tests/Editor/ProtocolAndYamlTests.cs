using System.Collections.Generic;
using System.Text;
using System.Text.RegularExpressions;
using NUnit.Framework;
using SimCore.Rpc;
using UnityEngine;
using UnityEngine.TestTools;
using Yaml.Drone;

namespace Nino.QuadSim.Tests
{
    public sealed class ProtocolAndYamlTests
    {
        [Test]
        public void ConnectDataAdvertisesVersionedContracts()
        {
            Dictionary<string, object> data = RpcCapabilities.ConnectData(
                5555, 5556, "127.0.0.1");

            Assert.That(data["protocol_major"], Is.EqualTo(1));
            Assert.That(data["unity_package_version"], Is.EqualTo("0.3.0"));
            Assert.That(data["frame_schema_version"], Is.EqualTo(1));
            Assert.That(data["time_schema_version"], Is.EqualTo(1));
            Assert.That(data["bind_address"], Is.EqualTo("127.0.0.1"));
        }

        [Test]
        public void SchemaV1ParsesAndIsRetained()
        {
            bool parsed = Parse(MinimalYaml(1), out ModelType model);

            Assert.That(parsed, Is.True);
            Assert.That(model.SchemaVersion, Is.EqualTo(1));
        }

        [Test]
        public void UnsupportedSchemaIsRejected()
        {
            LogAssert.Expect(LogType.Error,
                "[Drone.YamlParser] Unsupported schema_version 2: test");

            Assert.That(Parse(MinimalYaml(2), out _), Is.False);
        }

        [Test]
        public void BlankMethodIsRejectedBeforeDispatch()
        {
            LogAssert.Expect(LogType.Warning,
                "[RpcProtocol] Request shape exceeds protocol limits.");
            byte[] request = { 0x81, 0xa6, 0x6d, 0x65, 0x74, 0x68, 0x6f, 0x64, 0xa0 };

            Assert.That(RpcSerializer.DeserializeRequest(request).HasValue, Is.False);
        }

        [Test]
        public void OversizedMethodIsRejectedBeforeDispatch()
        {
            LogAssert.Expect(LogType.Warning,
                "[RpcProtocol] Request shape exceeds protocol limits.");
            var bytes = new List<byte>
            {
                0x81, 0xa6, 0x6d, 0x65, 0x74, 0x68, 0x6f, 0x64, 0xd9, 65
            };
            bytes.AddRange(Encoding.ASCII.GetBytes(new string('a', 65)));

            Assert.That(RpcSerializer.DeserializeRequest(bytes.ToArray()).HasValue, Is.False);
        }

        [Test]
        public void MalformedMessagePackIsRejected()
        {
            LogAssert.Expect(LogType.Warning,
                new Regex("\\[RpcProtocol\\] Failed to deserialize request:.*"));

            Assert.That(RpcSerializer.DeserializeRequest(new byte[] { 0xc1 }).HasValue, Is.False);
        }

        private static bool Parse(string yaml, out ModelType model)
        {
            return YamlParser.LoadDroneConfigFromString(
                yaml, "test", out model, out _, out _, out _, out _,
                out _, out _, out _, out _);
        }

        private static string MinimalYaml(int schemaVersion) => $@"
schema_version: {schemaVersion}
name: Test
physical_properties:
  mass_kg: 1
  com_offset: {{x: 0, y: 0, z: 0}}
  inertia: {{x: 1, y: 1, z: 1}}
  arm_length: 0.2
propulsion:
  prop_diameter: 0.2
  max_rpm: 10000
  thrust_coef: 0.1
  torque_coef: 0.03
  torque_coefficient_convention: power_coefficient_2_pi
  propellers:
    - {{id: 0, x: 0.1, y: 0, z: 0.1, dir: -1}}
    - {{id: 1, x: 0.1, y: 0, z: -0.1, dir: 1}}
    - {{id: 2, x: -0.1, y: 0, z: 0.1, dir: 1}}
    - {{id: 3, x: -0.1, y: 0, z: -0.1, dir: -1}}
";
    }
}
