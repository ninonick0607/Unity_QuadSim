//using UnityEngine.Addres; // Required

using System;
using UnityEngine;

namespace Yaml.Drone
{
    public struct ModelType
    {
        public string Name;
        //public AssetReference BodyMesh;
        
    }

    public struct DroneParameters
    {
        public float Mass;
        public Vector3 CenterOfMass;
        public Vector3 Inertia;
        public float Arm_Length;
        public float LinearDrag_Coeff;
        public float AngularDrag_Coeff;
    }

    public struct RotorModel
    {
        public float Prop_Diameter;
        public float MaxRPM;
        public float ThrustCoeff;
        public float TorqueCoeff;

    }

    public struct RotorInstance
    {
        // FL, FR, BL, BR
        public int ID;
        public Vector3 Location_Body;
        // +1 = CW, -1 = CCW
        public int Spin_Dir;
    }
    
    public struct RotorModelInstance
    {
        public RotorModel Model;
        public RotorInstance[] Rotors;
        
        public RotorModelInstance(int numRotors, RotorModel model)
        {
            Model = model;
            Rotors = new RotorInstance[numRotors];
        }
    }

    public struct CamParameters
    {
        public float CamArm_Length;
        public float FOV;
        public float CamPitch;
        public Vector3 CamSocket_Offset;
    }

    public struct PID
    {
        public float P;
        public float I;
        public float D;
    }
    public struct AcroPID
    {
        public PID AcroRoll {get;set;}
        public PID AcroPitch {get;set;}
        public PID AcroYaw {get;set;}
    }
    public struct AnglePID
    {
        public PID AngleRoll {get;set;}
        public PID AnglePitch {get;set;}
        public PID AngleYaw {get;set;}
    }
    public struct VelocityPID
    {
        public PID VelX {get;set;}
        public PID VelY {get;set;}
        public PID VelZ {get;set;}
    }    
    public struct PositionPID
    {
        public PID PosX {get;set;}
        public PID PosY {get;set;}
        public PID PosZ {get;set;}
    }

    public struct FlightParameters
    {
        public float MaxVelXY;
        public float MaxVelZ;
        public float MaxAngle;
        public float MaxRateRollPitch;
        public float MaxRateYaw;
        public float MaxThrust;
        public float MaxPID;
        
        // Nav
        public float AcceptableDistance;
    }

    // TODO: Not in use at the moment, make sure to implement down the line
    public struct ObstacleParameters
    {
        public Vector3 OuterBoundary;
        public Vector3 InnerBoundary;
        public Vector3 SpawnLocation;
    }

    public struct DroneConfig
    {
        public ModelType ModelParams;
        public DroneParameters DroneParams;
        public RotorModelInstance RotorParams;
        public CamParameters CamParams;
        public FlightParameters FlightParams;
        
        public AcroPID Acro;
        public AnglePID Angle;
        public VelocityPID Velocity;
        public PositionPID Position;
        
        // Source File TODO: Lets try and implement this
        public string SourceFile;
        
        // Should add isValid or unnecessary? 
    }
    
    // Math Stuff for Rotors
    public struct RotorPhysicsDerived
    {
        private float RevPerSec;
        private float MaxOmega;
        private float MaxOmegaSqr;
        private float MaxThrust;
        private float MaxTorque;
        private float MaxRollTorqueBody;
        private float MaxPitchTorqueBody;
        private float MaxYawTorqueBody;

        private const float AirDensity = 1.225f;

        private  RotorModel Model;
        private float ArmLengthMeters;
        
        // TODO: Out InModel? 
        public RotorPhysicsDerived(RotorModel InModel,float InArmLengthMeters)
        {
            Model = InModel;
            ArmLengthMeters = InArmLengthMeters;
            
            RevPerSec = Model.MaxRPM / 60.0f;
            MaxOmega = RevPerSec * 2.0f * (float)Math.PI;
            MaxOmegaSqr = MaxOmega * MaxOmega;
            
            float n2 = RevPerSec * RevPerSec;
            float D = Model.Prop_Diameter;
            
            // Thrust  = Ct * rho * n^2 * D^4
            MaxThrust = Model.ThrustCoeff * AirDensity * n2 * (float)Math.Pow(D, 4);
            // Torque = Cp * rho * n^2 * D^5/2PI
            MaxTorque = Model.TorqueCoeff * AirDensity * n2 * (((float)Math.Pow(D, 5))/ ((float)(2.5 * Math.PI)));
            
            // TODO: Original code says MaxThrust instead of MaxTorque, veriy this
            MaxRollTorqueBody = MaxTorque * ArmLengthMeters;
            MaxPitchTorqueBody = MaxTorque * ArmLengthMeters;
            
            // TODO: Hardcoded max yaw torque for 4 motors
            MaxYawTorqueBody = 4.0f * MaxTorque;
        }

    }
    
}
