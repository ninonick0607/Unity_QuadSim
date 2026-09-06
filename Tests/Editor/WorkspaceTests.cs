using NUnit.Framework;
using QuadSim.RuntimeUI.Workspace;
using UnityEngine;
using UnityEngine.UIElements;

namespace Nino.QuadSim.Tests
{
    public sealed class WorkspaceTests
    {
        [Test]
        public void FlightInitializationUsesEnuAltitudeAndThrottlePacking()
        {
            var pos = WorkspaceFlightSession.InitialCommand(global::SimCore.Common.GoalMode.Position, new Vector3(1, 2, 3), 45, 0.4f);
            Assert.That(pos.X, Is.EqualTo(1)); Assert.That(pos.Y, Is.EqualTo(2));
            Assert.That(pos.Z, Is.EqualTo(45)); Assert.That(pos.W, Is.EqualTo(3));
            var angle = WorkspaceFlightSession.InitialCommand(global::SimCore.Common.GoalMode.Angle, Vector3.zero, 0, 0.4f);
            Assert.That(angle.W, Is.EqualTo(0.4f));
        }
        [Test]
        public void PidHistoryRejectsEntireMalformedRow()
        {
            var cells = new string[37]; cells[0] = "2026-01-01 12:00:00";
            for (int i = 1; i < cells.Length; i++) cells[i] = "0.25";
            Assert.That(WorkspacePidHistory.TryParse(string.Join(",", cells), out var values), Is.True);
            Assert.That(values.Length, Is.EqualTo(36));
            cells[36] = "NaN";
            Assert.That(WorkspacePidHistory.TryParse(string.Join(",", cells), out values), Is.False);
            Assert.That(values, Is.Null);
            Assert.That(WorkspacePidHistory.TryParse("partial,1,2,3", out _), Is.False);
            cells[36] = "-1";
            Assert.That(WorkspacePidHistory.TryParse(string.Join(",", cells), out _), Is.False);
        }
        [Test]
        public void LevelOriginRejectsNonFiniteAndOutOfRangeCoordinates()
        {
            Assert.That(WorkspaceLevelService.ValidOrigin(48, 9, 450), Is.True);
            Assert.That(WorkspaceLevelService.ValidOrigin(91, 9, 450), Is.False);
            Assert.That(WorkspaceLevelService.ValidOrigin(48, double.NaN, 450), Is.False);
            Assert.That(WorkspaceLevelService.ValidOrigin(48, 9, double.PositiveInfinity), Is.False);
        }
        [TestCase(20, 300, WorkspaceDock.Left)]
        [TestCase(980, 300, WorkspaceDock.Right)]
        [TestCase(500, 580, WorkspaceDock.Bottom)]
        [TestCase(500, 300, WorkspaceDock.Floating)]
        [TestCase(-1, 300, WorkspaceDock.Floating)]
        [TestCase(1001, 300, WorkspaceDock.Floating)]
        [TestCase(20, 590, WorkspaceDock.Left)]
        public void SnapZonesUseWorkspaceCoordinatesAndDeterministicCornerPriority(float x, float y, WorkspaceDock expected)
        {
            Assert.That(WorkspaceDocking.Resolve(new Vector2(x, y), new Vector2(1000, 600)), Is.EqualTo(expected));
        }

        [Test]
        public void InactiveDockTabRetainsContentButStopsRefreshing()
        {
            var region = new VisualElement();
            var window = new WorkspaceWindow("test", "Test", new VisualElement(), (w, _) => region.Add(w));
            window.Place(WorkspaceDock.Right);
            window.SetPresented(false);
            Assert.That(window.IsOpen, Is.True);
            Assert.That(window.ShouldRefresh, Is.False);
            window.Open();
            Assert.That(window.ShouldRefresh, Is.True);
            Assert.That(window.Q<DropdownField>(), Is.Null, "Panel positioning has no configuration menu");
        }
        [Test]
        public void DetachedBackendRejectsPoseMutations()
        {
            using var backend = new WorkspaceBackend();
            Assert.That(backend.Ready, Is.False);
            Assert.That(backend.ApplyPose("missing", Vector3.zero, Vector3.zero), Is.False);
            Assert.That(backend.ApplyPose(null, new Vector3(float.NaN, 0, 0), Vector3.zero), Is.False);
        }

        [Test]
        public void LiveReadoutsKeepCollapsedGroupsAndReuseRowsAcrossValueUpdates()
        {
            var readout = new WorkspaceReadout();
            readout.SetReport("GPS\nAltitude   3.0 m\nSpeed   1.0 m/s");
            var group = readout.Q<Foldout>(); group.value = false;
            var value = readout.Q<Label>(className: "property-value");
            readout.SetReport("GPS\nAltitude   4.0 m\nSpeed   2.0 m/s");
            Assert.That(readout.Q<Foldout>(), Is.SameAs(group));
            Assert.That(group.value, Is.False);
            Assert.That(readout.Q<Label>(className: "property-value"), Is.SameAs(value));
            Assert.That(value.text, Is.EqualTo("4.0 m"));
            readout.SetReport("No selected drone sensor data.");
            Assert.That(readout.Q<Label>(className: "property-value"), Is.Null);
        }

        [Test]
        public void ClosedAndCollapsedPanelsStopPresentationRefreshAndCanBeRecovered()
        {
            var dock = new VisualElement();
            var window = new WorkspaceWindow("telemetry", "Telemetry", new VisualElement(), (w, _) => dock.Add(w));
            window.Place(WorkspaceDock.Right);
            Assert.That(window.ShouldRefresh, Is.True);
            window.ToggleExpanded();
            Assert.That(window.ShouldRefresh, Is.False);
            window.Close();
            window.Open();
            Assert.That(window.ShouldRefresh, Is.False, "Reopening retains collapse state");
            window.ToggleExpanded();
            Assert.That(window.ShouldRefresh, Is.True);
            Assert.That(window.parent, Is.SameAs(dock));
        }

        [Test]
        public void DockChangesReparentOneWindowAndPreserveItsContents()
        {
            var dock = new VisualElement(); var floating = new VisualElement();
            var window = new WorkspaceWindow("inspector", "Inspector", floating,
                (w, place) => (place == WorkspaceDock.Floating ? floating : dock).Add(w));
            var field = new TextField { value = "unsaved input" }; window.Body.Add(field);
            window.Place(WorkspaceDock.Left);
            window.Place(WorkspaceDock.Floating);
            Assert.That(dock.childCount, Is.Zero);
            Assert.That(floating.childCount, Is.EqualTo(1));
            window.Place(WorkspaceDock.Right);
            Assert.That(floating.childCount, Is.Zero);
            Assert.That(field.value, Is.EqualTo("unsaved input"));
        }

    }
}

