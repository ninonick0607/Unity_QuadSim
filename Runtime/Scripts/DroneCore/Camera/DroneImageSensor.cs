using System;
using System.Collections.Generic;
using RobotCore;
using UnityEngine;

namespace DroneCore.CameraSystem
{
    [DisallowMultipleComponent]
    public sealed class DroneImageSensor : MonoBehaviour
    {
        private readonly RaycastHit[] _depthHits = new RaycastHit[16];

        private QuadPawn _owner;
        private DroneCameraRig _rig;
        private Camera _camera;
        private RenderTexture _renderTexture;
        private Texture2D _readback;
        private int _sequence;

        public void ResetSequence() => _sequence = 0;

        public void Initialize(QuadPawn owner, DroneCameraRig rig)
        {
            _owner = owner;
            _rig = rig;
        }

        public Dictionary<string, object> Capture(
            string imageType,
            int width,
            int height,
            string encoding,
            int quality,
            float fieldOfViewDeg,
            float maxDepthM)
        {
            EnsureCamera();
            _camera.fieldOfView = fieldOfViewDeg;
            _camera.farClipPlane = maxDepthM;

            byte[] data;
            string pixelFormat;
            string dtype;
            int channels;

            if (imageType == "depth")
            {
                data = CaptureDepth(width, height, fieldOfViewDeg, maxDepthM);
                pixelFormat = "depth_m";
                dtype = "float32";
                channels = 1;
            }
            else
            {
                CaptureColor(width, height);
                if (imageType == "gray")
                {
                    byte[] gray = ToGray(_readback.GetPixels32());
                    if (encoding == "raw")
                    {
                        data = gray;
                        pixelFormat = "gray8";
                        channels = 1;
                    }
                    else
                    {
                        Color32[] pixels = new Color32[gray.Length];
                        for (int i = 0; i < gray.Length; i++)
                            pixels[i] = new Color32(gray[i], gray[i], gray[i], 255);
                        _readback.SetPixels32(pixels);
                        _readback.Apply(false, false);
                        data = Encode(_readback, encoding, quality);
                        pixelFormat = "rgb8_gray";
                        channels = 3;
                    }
                }
                else
                {
                    data = encoding == "raw"
                        ? ToRgb(_readback.GetPixels32())
                        : Encode(_readback, encoding, quality);
                    pixelFormat = "rgb8";
                    channels = 3;
                }
                dtype = "uint8";
            }

            _sequence++;
            SensorData metadata = _owner != null && _owner.Sensors != null
                ? _owner.Sensors.Latest
                : default;
            return new Dictionary<string, object>
            {
                ["agent_id"] = _owner != null ? _owner.AgentId : string.Empty,
                ["image_type"] = imageType,
                ["width"] = width,
                ["height"] = height,
                ["channels"] = channels,
                ["dtype"] = dtype,
                ["pixel_format"] = pixelFormat,
                ["encoding"] = encoding,
                ["origin"] = "bottom_left",
                ["field_of_view_deg"] = fieldOfViewDeg,
                ["max_depth_m"] = maxDepthM,
                ["episode_id"] = metadata.EpisodeId,
                ["step_index"] = metadata.StepIndex,
                ["sim_time_ns"] = metadata.SimTimeNanos,
                ["sim_time"] = metadata.SimTimeNanos * 1e-9,
                ["source_frame"] = "camera_optical_unity",
                ["sequence"] = _sequence,
                ["data"] = data,
            };
        }

        private void EnsureCamera()
        {
            if (_camera != null) return;
            if (_rig == null) _rig = GetComponent<DroneCameraRig>();

            var cameraObject = new GameObject("TrainingCamera");
            Transform parent = _rig != null ? _rig.FpvMount : transform;
            cameraObject.transform.SetParent(parent, false);
            _camera = cameraObject.AddComponent<Camera>();
            _camera.enabled = false;
            _camera.nearClipPlane = 0.05f;
            _camera.clearFlags = CameraClearFlags.Skybox;
            _camera.allowHDR = false;
            _camera.allowMSAA = false;
        }

        private void CaptureColor(int width, int height)
        {
            EnsureColorBuffers(width, height);
            RenderTexture previous = RenderTexture.active;
            RenderTexture previousTarget = _camera.targetTexture;
            _camera.targetTexture = _renderTexture;
            _camera.Render();
            RenderTexture.active = _renderTexture;
            _readback.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
            _readback.Apply(false, false);
            _camera.targetTexture = previousTarget;
            RenderTexture.active = previous;
        }

        private byte[] CaptureDepth(int width, int height, float verticalFovDeg, float maxDepthM)
        {
            float[] depth = new float[width * height];
            float verticalTan = Mathf.Tan(verticalFovDeg * Mathf.Deg2Rad * 0.5f);
            float aspect = width / (float)height;
            Transform cameraTransform = _camera.transform;
            Vector3 origin = cameraTransform.position;

            for (int y = 0; y < height; y++)
            {
                float v = ((y + 0.5f) / height * 2f - 1f) * verticalTan;
                for (int x = 0; x < width; x++)
                {
                    float u = ((x + 0.5f) / width * 2f - 1f) * verticalTan * aspect;
                    Vector3 direction = cameraTransform.TransformDirection(new Vector3(u, v, 1f).normalized);
                    int hitCount = Physics.RaycastNonAlloc(
                        origin, direction, _depthHits, maxDepthM, ~0, QueryTriggerInteraction.Ignore);

                    float nearest = maxDepthM;
                    for (int i = 0; i < hitCount; i++)
                    {
                        Collider collider = _depthHits[i].collider;
                        if (collider == null || collider.GetComponentInParent<QuadPawn>() == _owner)
                            continue;
                        if (_depthHits[i].distance < nearest)
                            nearest = _depthHits[i].distance;
                    }
                    depth[y * width + x] = nearest;
                }
            }

            byte[] bytes = new byte[depth.Length * sizeof(float)];
            Buffer.BlockCopy(depth, 0, bytes, 0, bytes.Length);
            return bytes;
        }

        private void EnsureColorBuffers(int width, int height)
        {
            if (_renderTexture != null
                && _renderTexture.width == width
                && _renderTexture.height == height)
                return;

            ReleaseColorBuffers();
            _renderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32)
            {
                name = $"{name}_TrainingCapture",
                antiAliasing = 1,
            };
            _renderTexture.Create();
            _readback = new Texture2D(width, height, TextureFormat.RGB24, false, false);
        }

        private static byte[] ToRgb(Color32[] pixels)
        {
            byte[] rgb = new byte[pixels.Length * 3];
            for (int i = 0; i < pixels.Length; i++)
            {
                int offset = i * 3;
                rgb[offset] = pixels[i].r;
                rgb[offset + 1] = pixels[i].g;
                rgb[offset + 2] = pixels[i].b;
            }
            return rgb;
        }

        private static byte[] ToGray(Color32[] pixels)
        {
            byte[] gray = new byte[pixels.Length];
            for (int i = 0; i < pixels.Length; i++)
                gray[i] = (byte)((77 * pixels[i].r + 150 * pixels[i].g + 29 * pixels[i].b) >> 8);
            return gray;
        }

        private static byte[] Encode(Texture2D texture, string encoding, int quality)
        {
            return encoding == "jpeg"
                ? texture.EncodeToJPG(quality)
                : texture.EncodeToPNG();
        }

        private void OnDestroy()
        {
            ReleaseColorBuffers();
            if (_camera != null) Destroy(_camera.gameObject);
        }

        private void ReleaseColorBuffers()
        {
            if (_renderTexture != null)
            {
                _renderTexture.Release();
                Destroy(_renderTexture);
                _renderTexture = null;
            }
            if (_readback != null)
            {
                Destroy(_readback);
                _readback = null;
            }
        }
    }
}
