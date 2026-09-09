using MathUtil;
using NUnit.Framework;
using QuadSim.RuntimeUI.Workspace;
using UnityEngine;

namespace Nino.QuadSim.Tests
{
    public sealed class WorkspaceAxesTests
    {
        [Test]
        public void WorldNedSwapsNorthEastWhileBodyFrdKeepsForward()
        {
            Assert.That(WorkspaceAxes.Direction(0, SimFrame.FLU, false, Quaternion.identity), Is.EqualTo(Vector3.right));
            Assert.That(WorkspaceAxes.Direction(1, SimFrame.FLU, false, Quaternion.identity), Is.EqualTo(Vector3.forward));
            Assert.That(WorkspaceAxes.Direction(2, SimFrame.FLU, false, Quaternion.identity), Is.EqualTo(Vector3.up));
            Assert.That(WorkspaceAxes.Direction(0, SimFrame.FRD, false, Quaternion.identity), Is.EqualTo(Vector3.forward));
            Assert.That(WorkspaceAxes.Direction(1, SimFrame.FRD, false, Quaternion.identity), Is.EqualTo(Vector3.right));
            Assert.That(WorkspaceAxes.Direction(2, SimFrame.FRD, false, Quaternion.identity), Is.EqualTo(Vector3.down));
            Assert.That(WorkspaceAxes.Direction(0, SimFrame.FRD, true, Quaternion.identity), Is.EqualTo(Vector3.right));
            Assert.That(WorkspaceAxes.Direction(1, SimFrame.FRD, true, Quaternion.identity), Is.EqualTo(Vector3.back));
            Assert.That(WorkspaceAxes.Direction(2, SimFrame.FRD, true, Quaternion.identity), Is.EqualTo(Vector3.down));
        }
        [Test]
        public void BodyAxesFollowEntityRotationAndWorldAxesDoNot()
        {
            var rotation = Quaternion.Euler(20, 90, 30);
            for (int i = 0; i < 3; i++)
            {
                var world = WorkspaceAxes.Direction(i, SimFrame.FLU, false, Quaternion.identity);
                Assert.That(WorkspaceAxes.Direction(i, SimFrame.FLU, false, rotation), Is.EqualTo(world));
                var body = WorkspaceAxes.Direction(i, SimFrame.FLU, true, rotation);
                Assert.That(Vector3.Distance(body, rotation * world), Is.LessThan(.0001f));
            }
        }
    }
}
