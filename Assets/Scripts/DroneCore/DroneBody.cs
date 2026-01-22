using System;
using UnityEngine;

namespace QuadSim.DroneCore
{
    public sealed class DroneBody : MonoBehaviour
    {
        [Header("Core")]
        [SerializeField] private Rigidbody rb;

        [Header("Motor Transforms (optional; auto-found if empty)")]
        [SerializeField] private Transform motorFL;
        [SerializeField] private Transform motorFR;
        [SerializeField] private Transform motorBL;
        [SerializeField] private Transform motorBR;

        // Canonical motor order used everywhere: [0]=FL, [1]=FR, [2]=BL, [3]=BR
        private readonly Transform[] _motors = new Transform[4];

        public Rigidbody Rigidbody => rb;

        public Transform MotorFL => _motors[0];
        public Transform MotorFR => _motors[1];
        public Transform MotorBL => _motors[2];
        public Transform MotorBR => _motors[3];

        public Transform[] Motors => _motors;

        private void Reset()
        {
            rb = GetComponent<Rigidbody>();
        }

        private void Awake()
        {
            AutoWireIfNeeded();
            ValidateOrThrow();
        }

        public void AutoWireIfNeeded()
        {
            if (rb == null) rb = GetComponent<Rigidbody>();

            // Prefer explicit references if set; otherwise search by name.
            if (motorFL == null) motorFL = FindDeepChild(transform, "MotorFL");
            if (motorFR == null) motorFR = FindDeepChild(transform, "MotorFR");
            if (motorBL == null) motorBL = FindDeepChild(transform, "MotorBL");
            if (motorBR == null) motorBR = FindDeepChild(transform, "MotorBR");

            _motors[0] = motorFL;
            _motors[1] = motorFR;
            _motors[2] = motorBL;
            _motors[3] = motorBR;
        }

        public void ValidateOrThrow()
        {
            if (rb == null) throw new Exception($"{name}: DroneBody requires a Rigidbody reference.");
            for (int i = 0; i < 4; i++)
            {
                if (_motors[i] == null) throw new Exception($"{name}: Missing motor transform for index {i} (FL,FR,BL,BR).");
            }
        }

        private static Transform FindDeepChild(Transform root, string childName)
        {
            // BFS search for stable behavior.
            var q = new System.Collections.Generic.Queue<Transform>();
            q.Enqueue(root);
            while (q.Count > 0)
            {
                var t = q.Dequeue();
                if (t.name == childName) return t;
                for (int i = 0; i < t.childCount; i++) q.Enqueue(t.GetChild(i));
            }
            return null;
        }
    }
}
