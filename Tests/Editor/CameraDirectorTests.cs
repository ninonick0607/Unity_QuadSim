using DroneCore.CameraSystem;
using NUnit.Framework;
using UnityEngine;

namespace Nino.QuadSim.Tests
{
    public sealed class CameraDirectorTests
    {
        private GameObject host, entity;
        private CameraDirector director;
        [SetUp] public void Setup()
        {
            host = new GameObject("Camera test");
            entity = GameObject.CreatePrimitive(PrimitiveType.Cube);
            entity.transform.position = new Vector3(5, 3, 7);
            director = host.AddComponent<CameraDirector>();
        }
        [TearDown] public void Cleanup()
        {
            Object.DestroyImmediate(host);
            if (entity != null) Object.DestroyImmediate(entity);
        }
        [Test]
        public void ReleasingTargetKeepsOrbitStationaryDuringObjectEdits()
        {
            director.FrameTarget(entity.transform);
            var position = director.ViewCamera.transform.position;
            var rotation = director.ViewCamera.transform.rotation;
            director.ReleaseTarget();
            entity.transform.position += new Vector3(4, 6, 8);
            typeof(CameraDirector).GetMethod("UpdateCameraPose", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)
                .Invoke(director, new object[] { 1f });
            Assert.That(director.Target, Is.Null);
            Assert.That(director.Mode, Is.EqualTo(CameraViewMode.Orbit));
            Assert.That(Vector3.Distance(position, director.ViewCamera.transform.position), Is.LessThan(.001f));
            Assert.That(Quaternion.Angle(rotation, director.ViewCamera.transform.rotation), Is.LessThan(.001f));
        }
        [Test] public void GenericTargetDoesNotOwnDisplayCamera()
        {
            var sensor = new GameObject("Sensor", typeof(Camera));
            sensor.transform.SetParent(entity.transform);
            director.BindEntity(entity.transform);
            Assert.That(director.Target, Is.EqualTo(entity.transform));
            Assert.That(director.ViewCamera.transform.parent, Is.EqualTo(host.transform));
            Assert.That(sensor.transform.parent, Is.EqualTo(entity.transform));
            Assert.That(sensor.GetComponent<Camera>().enabled, Is.True);
            Object.DestroyImmediate(entity);
            Assert.That(director.ViewCamera, Is.Not.Null);
        }
        [Test] public void FrameBoundsFitsAndKeepsTargetCentered()
        {
            entity.transform.localScale = new Vector3(10, 2, 4);
            director.FrameTarget(entity.transform);
            var camera = director.ViewCamera;
            var p = camera.WorldToViewportPoint(entity.transform.position);
            Assert.That(p.x, Is.EqualTo(.5f).Within(.001f));
            Assert.That(p.y, Is.EqualTo(.5f).Within(.001f));
            Assert.That(p.z, Is.GreaterThan(5));
            foreach (var corner in new[] { new Vector3(5, 1, 2), new Vector3(-5, -1, -2) })
            {
                var point = camera.WorldToViewportPoint(entity.transform.position + corner);
                Assert.That(point.x, Is.InRange(0f, 1f));
                Assert.That(point.y, Is.InRange(0f, 1f));
            }
        }
        [Test] public void AxisAndProjectionPreserveTargetAndFlyIsPerspective()
        {
            director.FrameTarget(entity.transform);
            director.AxisView(Vector3.down);
            Assert.That(Vector3.Dot(director.ViewCamera.transform.forward, Vector3.down), Is.GreaterThan(.999f));
            Assert.That(director.Orthographic, Is.True);
            director.SetMode(CameraViewMode.Free);
            Assert.That(director.Orthographic, Is.False);
            Assert.That(director.Target, Is.EqualTo(entity.transform));
        }
        [Test] public void PanMovesPivotAndZoomCannotCrossIt()
        {
            director.FrameTarget(entity.transform);
            var before = director.FocusPoint;
            director.Navigate(new Vector2(20, 10), true, false, 800);
            Assert.That(Vector3.Distance(before, director.FocusPoint), Is.GreaterThan(0));
            director.Zoom(10000);
            Assert.That(director.FollowDistance, Is.GreaterThan(0));
        }
        [Test] public void AttachedViewUsesExplicitMount()
        {
            var mount = new GameObject("Mount").transform;
            mount.SetParent(entity.transform);
            mount.localPosition = new Vector3(0, 2, 0);
            mount.rotation = Quaternion.Euler(12, 37, 5);
            director.BindEntity(entity.transform, mount);
            director.SetMode(CameraViewMode.FPV);
            Assert.That(Vector3.Distance(director.ViewCamera.transform.position, mount.position), Is.LessThan(.001f));
            Assert.That(Quaternion.Angle(director.ViewCamera.transform.rotation, mount.rotation), Is.LessThan(.001f));
        }
        [TestCase(1,0,0)] [TestCase(-1,0,0)]
        [TestCase(0,1,0)] [TestCase(0,-1,0)]
        [TestCase(0,0,1)] [TestCase(0,0,-1)]
        public void FreeMovementUsesCameraAxesAndWorldVerticalWhilePaused(int x, int y, int z)
        {
            director.BindEntity(entity.transform);
            director.SetMode(CameraViewMode.Free);
            var camera = director.ViewCamera;
            camera.transform.rotation = Quaternion.Euler(25, 40, 0);
            var before = camera.transform.position;
            var expected = camera.transform.right*x + Vector3.up*y + camera.transform.forward*z;
            float timeScale = Time.timeScale;
            try {
                Time.timeScale = 0;
                director.FreeMoveSpeed = 4;
                director.AddFreeMovementInput(new Vector3(x,y,z), false);
                Tick(.25f);
                Assert.That(Vector3.Distance(camera.transform.position-before, expected), Is.LessThan(.0001f));
                var stopped=camera.transform.position;
                Tick(.25f);
                Assert.That(camera.transform.position, Is.EqualTo(stopped), "Released input must not keep moving");
            } finally { Time.timeScale=timeScale; }
        }
        [Test] public void BoostAndDiagonalMovementHaveBoundedSpeed()
        {
            director.BindEntity(entity.transform); director.SetMode(CameraViewMode.Free);
            director.ViewCamera.transform.rotation=Quaternion.identity;
            director.FreeMoveSpeed=2;
            var before=director.ViewCamera.transform.position;
            director.AddFreeMovementInput(new Vector3(1,0,1).normalized,true); Tick(.5f);
            Assert.That(Vector3.Distance(before,director.ViewCamera.transform.position),Is.EqualTo(3).Within(.001f));
        }
        private void Tick(float dt) => typeof(CameraDirector).GetMethod("UpdateCameraPose",System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.NonPublic).Invoke(director,new object[]{dt});
    }
}
