using System;
using UnityEngine;

namespace RobotCore.Common
{
    [Serializable]
    public struct Axis4
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        public Axis4(float x, float y, float z, float w)
        {
            X = x; Y = y; Z = z; W = w;
        }

        // Vector views
        public Vector3 XYZ
        {
            get => new Vector3(X, Y, Z);
            set { X = value.x; Y = value.y; Z = value.z; }
        }

        // Semantic aliases (for Angle/Rate modes)
        public float Roll  { get => X; set => X = value; }
        public float Pitch { get => Y; set => Y = value; }
        public float Yaw   { get => Z; set => Z = value; }

        // Semantic alias for W depending on mode
        public float Throttle { get => W; set => W = value; }   // Rate mode
        public float AltOrVz  { get => W; set => W = value; }   // Position/Velocity/Angle modes

        public float this[int index]
        {
            get => index switch
            {
                0 => X, 1 => Y, 2 => Z, 3 => W,
                _ => throw new IndexOutOfRangeException($"Axis4 index out of bounds: {index}")
            };
            set
            {
                switch (index)
                {
                    case 0: X = value; break;
                    case 1: Y = value; break;
                    case 2: Z = value; break;
                    case 3: W = value; break;
                    default: throw new IndexOutOfRangeException($"Axis4 index out of bounds: {index}");
                }
            }
        }

        public void Zero() => X = Y = Z = W = 0f;
        public static Axis4 ZeroValue => new Axis4(0, 0, 0, 0);
    }
}