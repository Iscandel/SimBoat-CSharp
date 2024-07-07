using Assets.Scripts.Physics;
using Palmmedia.ReportGenerator.Core.CodeAnalysis;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements.Experimental;
using static Unity.VisualScripting.Member;

public class SimpleRudder : MonoBehaviour, IForceListener, IPhysicsListener
{
    private float _rudderAngle;         // degrees
    private Vector3 _rudderDirection;   // Body Ned

    private IDynamics _actuatorDynamics;

    private IWaterProvider _waterProvider;
    private IPhysicsManager _physicsManager;

    private EntityEnvironment _environment;

    private IBody _body;
    private BodyState _state;           // NED

    public float _maxRudderAngle;       // Degrees
    public Vector3 _rudderAppliPoint;   // NED, relative to ship

    public float _area;

    public float _scale;

    public float _heightOffset;
    public bool _simplifiedHeightDetection;

    public float _torqueFactor;

    public bool _isDebug;

    [Tooltip("Angle / North"), Range(0, 360f)]
    public float _heading;

    // Start is called before the first frame update
    void Start()
    {
        _simplifiedHeightDetection = true;
        //_heightOffset = 0;

        GameObject[] oceanGO = GameObject.FindGameObjectsWithTag("Ocean");
        _waterProvider = oceanGO[0].GetComponent<IWaterProvider>();

        GameObject[] physicsManager = GameObject.FindGameObjectsWithTag("PhysicsManager");
        _physicsManager = physicsManager[0].GetComponent<IPhysicsManager>();
        _physicsManager.AddPhysicsEventListener(this);

        MeshBasedWaterPhysics boatForcesUnity = GetComponent<MeshBasedWaterPhysics>();

        if (boatForcesUnity.Body != null)
        {
            _body = boatForcesUnity.Body;
            // We provide forces in NED
            _physicsManager.AddForceListener(this, _body, RefFrame.BODY_NED);
        }

        const float absSpeed = 20;
        _actuatorDynamics = new LinearActuatorDynamics(absSpeed);

        _environment = GetComponent<EntityEnvironment>();
    }

    // Update is called once per frame
    void Update()
    {       
        float setpoint = 0;
        float angle;
        if (Input.GetKey(KeyCode.D))
            angle = -1;
        else if (Input.GetKey(KeyCode.Q))
            angle = 1;
        else
            angle = -Input.GetAxis("Horizontal");

        setpoint = angle * _maxRudderAngle;
        
        _actuatorDynamics.SetSetpoint(setpoint);

        //if (_isDebug)
        //{
        //    var apparentWater = -(Quaternion.Euler(0, _heading, 0) * Vector3.forward);
        //    //apparentWater.y = -1;
        //    var liftDir = MathTools.VectorNEDToUnity(calculateLiftDirection(
        //        Quaternion.Inverse(_state.rotation) * MathTools.VectorUnityToNED(apparentWater), new Vector3(1,0,0)));

        //    Vector3 appliPointUnity = MathTools.NEDToUnity(_state.worldCenterOfMass + _state.rotation * _rudderAppliPoint);
        
        //    DrawArrow.ForDebug(appliPointUnity, apparentWater.normalized, Color.green);
        //    DrawArrow.ForDebug(appliPointUnity, _state.rotation * liftDir.normalized, Color.red);
        //}
    }

    private void FixedUpdate()
    {
        _actuatorDynamics.Update(Time.fixedDeltaTime);
        _rudderAngle = _actuatorDynamics.GetState();
        _rudderDirection = MathTools.VectorUnityToNED(Quaternion.Euler(0, _rudderAngle, 0) * Vector3.forward);
    }

    ForceTorque ComputeRudderForce()
    {
        ForceTorque res  = new ForceTorque();

        Vector3 appliPointUnity = MathTools.NEDToUnity(_state.worldCenterOfMass + _state.rotation * _rudderAppliPoint);   

        //float RHO = worldThrustPosition.y < _waterHeightPropellerPos ? UnityPhysicsConstants.RHO : UnityPhysicsConstants.RHO_AIR;

        float waterHeight;
        if (_simplifiedHeightDetection)
        {
            waterHeight = 0f;
        }
        else
        {
            float[] heights = new float[1];
            Vector3[] samples = new Vector3[1];
            samples[0] = appliPointUnity;
            if (_waterProvider.SampleHeightAt(samples, ref heights))
                waterHeight = heights[0];
            else
                waterHeight = 0;

        }

        //// NED velocity at rudder in local coords
        Vector3 velocityAtRudder = MathTools.VelocityAt_NEDToNED(_state.velocity_body, _state.angularVelocity_body, _rudderAppliPoint);
        float v = velocityAtRudder.magnitude;
        float rho = _environment.GetRho();
        Vector3 force = Vector3.zero;
        force.y = -0.5f * rho * _area * v * v * _rudderAngle / (float)_maxRudderAngle * _scale;
        res.torque = MathTools.TorqueNEDToNED(_rudderAppliPoint, force) * _scale;

        // Ensure minimum torque when turning
        float rudderAngle = -_rudderAngle;
        res.torque.z = res.torque.z < 0 ? Mathf.Min(res.torque.z, rudderAngle * _torqueFactor) :
                       res.torque.z > 0 ? Mathf.Max(res.torque.z, rudderAngle * _torqueFactor) : rudderAngle * _torqueFactor;

        if (_isDebug)
        {
            Vector3 unityDirection = (Quaternion.Euler(0, _rudderAngle, 0) * MathTools.QuaternionNEDtoUnity(_state.rotation)) * Vector3.back;
            Debug.DrawLine(appliPointUnity, appliPointUnity + unityDirection.normalized * 3, Color.magenta);
        }

        res.force.x = 35000;
        return res;
    }

    public void OnPhysicsEvent(IPhysicsListener.EventType eventType, object data)
    {
        if (eventType == IPhysicsListener.EventType.STATE_UPDATED)
        {
            // We want NED state
            _state = _physicsManager.GetBodyState(_body, RefFrame.NED);
        }
        else if (eventType == IPhysicsListener.EventType.BODY_CREATED)
        {
            _body = (IBody)data;
            _physicsManager.AddForceListener(this, _body, RefFrame.BODY_NED);
        }
    }

    public void ComputeForce(IBody body, ref ForceTorque force, BodyState state)
    {
        force = ComputeRudderForce();
    }
}

