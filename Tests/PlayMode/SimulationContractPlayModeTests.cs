using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using SimCore;
using UnityEngine;
using UnityEngine.TestTools;

namespace Nino.QuadSim.PlayModeTests
{
    public sealed class SimulationContractPlayModeTests
    {
        [UnityTest]
        public IEnumerator ManualStepsRunPrePhysicsThenPostPhysicsAndResetClock()
        {
            var root = new GameObject("SimulationContractTest");
            var manager = root.AddComponent<SimulationManager>();
            var probe = root.AddComponent<SimulationProbe>();
            manager.RegisterManagers(probe);
            manager.SetRunMode(SimulationManager.RunMode.Manual);
            manager.SetPaused(true);

            yield return null;

            manager.StepImmediate(2);
            Assert.That(manager.StepIndex, Is.EqualTo(2));
            Assert.That(manager.SimTimeNanos, Is.EqualTo(8_000_000));
            CollectionAssert.AreEqual(
                new[] { "pre", "post", "pre", "post" }, probe.Phases);

            manager.ResetSimulation();
            Assert.That(manager.StepIndex, Is.Zero);
            Assert.That(manager.SimTimeNanos, Is.Zero);
            Assert.That(probe.ResetCount, Is.EqualTo(1));

            Object.Destroy(root);
            yield return null;
        }

        [Test]
        public void SceneConfigDefaultsToLoopbackRpc()
        {
            var config = ScriptableObject.CreateInstance<QuadSimSceneConfig>();
            Assert.That(config.rpcBindAddress, Is.EqualTo("127.0.0.1"));
            Object.Destroy(config);
        }
    }

    public sealed class SimulationProbe : MonoBehaviour, ISimulatable
    {
        public readonly List<string> Phases = new List<string>();
        public int ResetCount { get; private set; }
        public int ExecutionOrder => 0;
        public string DebugName => nameof(SimulationProbe);

        public void OnSimulationStart(SimulationManager sim) { }
        public void OnSimulationReset(SimulationManager sim) => ResetCount++;
        public void PrePhysicsStep(double dtSec, long nowNanos) => Phases.Add("pre");
        public void PostPhysicsStep(double dtSec, long nowNanos) => Phases.Add("post");
    }
}
