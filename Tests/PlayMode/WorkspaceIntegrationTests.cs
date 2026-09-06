#if UNITY_EDITOR
using System.Collections;
using DroneCore;
using DroneCore.CameraSystem;
using DroneCore.Core;
using NUnit.Framework;
using QuadSim.RuntimeUI.Workspace;
using SimCore;
using SimCore.Common;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

namespace Nino.QuadSim.PlayModeTests
{
    public sealed class WorkspaceIntegrationTests
    {
        [UnityTest]
        public IEnumerator NativeCommandsSurviveFastStepsAndStopOnAuthorityChange()
        {
            var root = new GameObject("Workspace integration");
            WorkspaceBackend backend = null;
            QuadPawn first = null, second = null;
            string csv = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "quadsim-pid-test-" + System.Guid.NewGuid().ToString("N") + ".csv");
            try
            {
                var manager = root.AddComponent<SimulationManager>();
                manager.SetRunMode(SimulationManager.RunMode.Manual); manager.SetPaused(true);
                root.AddComponent<ControlAuthorityManager>();
                var sim = root.AddComponent<SimCoreApi>();
                var drones = root.AddComponent<DroneManager>();
                var prefab = AssetDatabase.LoadAssetAtPath<QuadPawn>("Packages/com.nino.quadsim/Runtime/Prefabs/Quad Prefabs/flightmare_pawn.prefab");
                Assert.That(prefab, Is.Not.Null);
                drones.SetSceneEnv(12345, false, 47, 8, 400, StartupSpawnPolicy.SceneDronesOnly, prefab, null, false, "", MathUtil.SimFrame.FLU, GoalMode.Velocity);
                var director = CameraDirector.GetOrCreate(root);
                first = drones.SpawnDroneWithConfig("flightmare_quad", new Vector3(0, 30, 0), Quaternion.identity);
                second = drones.SpawnDroneWithConfig("flightmare_quad", new Vector3(10, 30, 0), Quaternion.identity);
                Assert.That(first, Is.Not.Null); Assert.That(second, Is.Not.Null);
                backend = new WorkspaceBackend(); backend.Connect();
                yield return null;
                backend.SwitchToUI();
                backend.Select(first.AgentId);
                Assert.That(backend.SelectedId, Is.EqualTo(first.AgentId));
                backend.FollowSelected();
                Assert.That(director.TargetDrone, Is.SameAs(first));
                Assert.That(director.Mode, Is.EqualTo(CameraViewMode.Follow));
                var flight = backend.Flight;
                Assert.That(flight.EnterMode(first.AgentId, GoalMode.Velocity), Is.True);
                Assert.That(flight.SetAxis(first.AgentId, GoalMode.Velocity, 0, 1.25f), Is.True);
                // More than the proxy's 0.5 simulated-second stale threshold, without any UI Update.
                manager.StepImmediate(200);
                Assert.That(first.CommandProxy.GetCommandValue().X, Is.EqualTo(1.25f));
                Assert.That(first.CommandProxy.IsUsingFallback(), Is.False);
                backend.Select(second.AgentId);
                Assert.That(flight.SetAxis(first.AgentId, GoalMode.Velocity, 0, 4), Is.False, "Stale selected IDs must not command a different drone");
                Assert.That(flight.EnterMode(second.AgentId, GoalMode.Angle), Is.True);
                Assert.That(flight.SetAxis(second.AgentId, GoalMode.Angle, 3, float.NaN), Is.False);
                Assert.That(flight.SetAxis(second.AgentId, GoalMode.Angle, 3, 2), Is.True);
                Assert.That(flight.State.Command.W, Is.EqualTo(1));
                Assert.That(flight.SetFrame(second.AgentId, true), Is.True);
                Assert.That(second.Sensors.outputFrame, Is.EqualTo(MathUtil.SimFrame.FRD));
                Assert.That(flight.State.Engaged, Is.False, "Frame changes clear stale commands");
                Assert.That(flight.EnterMode(second.AgentId, GoalMode.Position), Is.True);
                sim.NotifyInternalConnected(); sim.RequestAuthority(InputSource.Internal);
                Assert.That(flight.CanCommand, Is.False);
                Assert.That(flight.EnterMode(second.AgentId, GoalMode.Rate), Is.False);
                sim.SwitchToUI();
                Assert.That(flight.State.Engaged, Is.False, "Authority restoration must not replay prior setpoints");
                Assert.That(backend.Level.Apply(55, true, 48, 9, 450, new Vector3(2, 4, 6)), Is.True);
                Assert.That(sim.Geo.OriginLatDeg, Is.EqualTo(48));
                Assert.That(first.Sensors.enableNoise, Is.True);
                Assert.That(drones.SpawnOrigin, Is.EqualTo(new Vector3(2, 4, 6)));
                var gains = WorkspacePidHistory.Capture(second.Controller);
                WorkspacePidHistory.Save(csv, second.Controller);
                var history = WorkspacePidHistory.Read(csv);
                Assert.That(history.Count, Is.EqualTo(1));
                second.Controller.SetPIDGainTerm(DroneCore.Controllers.CascadedController.EPIDStage.Position, 0, 0, gains[0] + 1, false);
                Assert.That(WorkspacePidHistory.Apply(second.Controller, history[0]), Is.True);
                Assert.That(WorkspacePidHistory.Capture(second.Controller)[0], Is.EqualTo(gains[0]).Within(0.00001f));
                Assert.That(WorkspacePidHistory.Apply(second.Controller, "broken,1,2"), Is.False);
                Assert.That(backend.Level.AddObject("cube", new Vector3(2, 2, 1), Vector3.one, true), Is.True);
                var objects = backend.Level.Objects;
                string firstObject = objects.ActiveObjects[0].ObjectId;
                Assert.That(backend.Level.AddObject("sphere", new Vector3(3, 2, 1), Vector3.one, true), Is.True);
                Assert.That(objects.ActiveObjects.Count, Is.EqualTo(2));
                Assert.That(backend.Level.RemoveObject(firstObject), Is.True);
                Assert.That(objects.ActiveObjects.Count, Is.EqualTo(1));
                var preservedPose = second.transform.position;
                Assert.That(backend.ReloadSelected(second.AgentId), Is.True);
                second = drones.SelectedDrone;
                Assert.That(drones.DroneCount, Is.EqualTo(2));
                Assert.That(Vector3.Distance(second.transform.position, preservedPose), Is.LessThan(0.001f));
                sim.NotifyExternalConnectionAttempt();
                Assert.That(backend.ApprovalPending, Is.True);
                backend.ApproveExternal(false);
                Assert.That(backend.ApprovalPending, Is.False);
                manager.ResetSimulation();
                Assert.That(flight.State.Engaged, Is.False);
            }
            finally
            {
                backend?.Dispose();
                if (first != null) Object.Destroy(first.gameObject);
                if (second != null) Object.Destroy(second.gameObject);
                Object.Destroy(root);
                if (System.IO.File.Exists(csv)) System.IO.File.Delete(csv);
            }
            yield return null;
        }
    }
}
#endif
