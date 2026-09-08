using DroneCore.Core;
using DroneCore.CameraSystem;
using NUnit.Framework;
using UnityEngine;
using Yaml.Drone;

namespace Nino.QuadSim.Tests
{
    public sealed class DronePrefabCatalogTests
    {
        [TestCase("standard_quad", "Standard_quad")]
        [TestCase("flightmare_quad", "flightmare_pawn")]
        [TestCase("crazyflie2_quad", "crazyflie_prefab")]
        public void ConfigLoadsDistinctPrefabWithForwardExternalFpvMount(string key, string prefabName)
        {
            Assert.That(DroneConfigLoader.LoadConfig(key,out var config),Is.True);
            var catalog=Resources.Load<DronePrefabCatalog>("QuadSim/DronePrefabCatalog");
            Assert.That(catalog,Is.Not.Null);
            Assert.That(config.ModelParams.PrefabKey,Is.EqualTo(key));
            Assert.That(config.ModelParams.BodyMesh,Is.Not.Empty);
            Assert.That(catalog.TryResolve(config,out var prefab,out _),Is.True);
            Assert.That(prefab.name,Is.EqualTo(prefabName));
            var mount=prefab.GetComponent<DroneCameraRig>().FpvMount;
            Assert.That(mount.localPosition.x,Is.GreaterThan(.02f));
            Assert.That(Vector3.Dot(mount.localRotation*Vector3.forward,Vector3.right),Is.GreaterThan(.999f));
        }
        [Test] public void ExplicitUnknownKeyDoesNotFallBackToKnownFilename()
        {
            Assert.That(DroneConfigLoader.LoadConfig("flightmare_quad",out var config),Is.True);
            config.ModelParams.PrefabKey="typo";
            Assert.That(Resources.Load<DronePrefabCatalog>("QuadSim/DronePrefabCatalog").TryResolve(config,out _,out _),Is.False);
        }
        [Test] public void LegacyConfigWithoutKeyStillResolvesBySourceName()
        {
            Assert.That(DroneConfigLoader.LoadConfig("crazyflie2_quad",out var config),Is.True);
            config.ModelParams.PrefabKey=null;
            Assert.That(Resources.Load<DronePrefabCatalog>("QuadSim/DronePrefabCatalog").TryResolve(config,out var prefab,out _),Is.True);
            Assert.That(prefab.name,Is.EqualTo("crazyflie_prefab"));
        }
    }
}
