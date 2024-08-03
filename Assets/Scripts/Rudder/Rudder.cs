using Sim.Physics;
using Palmmedia.ReportGenerator.Core.CodeAnalysis;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements.Experimental;
using static Unity.VisualScripting.Member;

public class Rudder : MonoBehaviour, IForceListener, IPhysicsListener
{
    private float _rudderAngle;         // degrees
    private Vector3 _rudderDirection;   // Body Ned

    private IDynamics _actuatorDynamics;

    private IWaterProvider _waterProvider;
    private IPhysicsManager _physicsManager;

    private IBody _body;
    private BodyState _state;           // NED

    private LiftDrag _liftDrag;

    public float _maxRudderAngle;       // Degrees
    public Vector3 _rudderAppliPoint;   // NED, relative to ship

    public float _area;
    public AnimationCurve _liftCurve;
    public AnimationCurve _dragCurve;

    //
    public AnimationCurve _liftCurveFxFy;
    public AnimationCurve _dragCurveFxFy;
    private LiftDrag _liftDrag2;

    //public float _scale;
    public float _forceScale;
    public float _torqueScale;

    public float _heightOffset;
    public bool _simplifiedHeightDetection;

    public float _torqueFactor;

    public bool _isDebug;

    [Tooltip("Angle / North"), Range(0, 360f)]
    public float _heading;

    EntityEnvironment _environment;

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

        _liftDrag = new LiftDrag();
        _liftDrag.LiftCurve = _liftCurve;
        _liftDrag.DragCurve = _dragCurve;
        _liftDrag.Area = _area;
        _liftDrag.ForceScale = _forceScale;
        _liftDrag.TorqueScale = _torqueScale;
        _liftDrag.AppliPoint = _rudderAppliPoint;
        _liftDrag.IsDebug = _isDebug;

        //
        _liftDrag2 = new LiftDrag();
        _liftDrag2.LiftCurve = _liftCurveFxFy;
        _liftDrag2.DragCurve = _dragCurveFxFy;
        _liftDrag2.Area = _area;
        _liftDrag2.ForceScale = _forceScale;
        _liftDrag2.TorqueScale = _torqueScale;
        _liftDrag2.AppliPoint = _rudderAppliPoint;
        _liftDrag2.IsDebug = _isDebug;

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
        //_actuatorDynamics.Update(Time.deltaTime);
        //_rudderAngle = _actuatorDynamics.GetState();
    }

    private void FixedUpdate()
    {
        _actuatorDynamics.Update(Time.fixedDeltaTime);
        _rudderAngle = _actuatorDynamics.GetState();
        _rudderDirection = MathTools.VectorUnityToNED(Quaternion.Euler(0, _rudderAngle, 0) * Vector3.forward);
    }

    ForceTorque ComputeRudderForce()
    {
        _liftDrag.IsDebug = _isDebug;
        _liftDrag2.IsDebug = _isDebug;

        ForceTorque res;// = new ForceTorque();

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
        //Matrix4x4 mat = Matrix4x4.TRS(_state.position, _state.rotation, new Vector3(1, 1, 1));

        //if (transform.TransformPoint(_thrustAppliPoint).y + _heightOffset > waterHeight)

        Vector3 fluidVector_body = Quaternion.Inverse(_state.rotation) * _environment.GetCurrentVector_WorldNED();

        // TODO Remove
        //fluidVector_body = MathTools.VectorUnityToNED(Quaternion.Euler(0, _heading, 0) * Vector3.forward);
        //Vector3 pos = MathTools.NEDToUnity(_state.worldCenterOfMass);
        //DrawArrow.ForDebug(pos, MathTools.VectorNEDToUnity(_state.rotation * fluidVector_body.normalized), Color.yellow);
        //
        
        res = _liftDrag.ComputeForce(_state, fluidVector_body, _rudderDirection, _environment.GetRho());

        //_liftDrag2.ComputeForce2(_state, fluidVector_body, _rudderDirection);

        // Ensure minimum torque when turning
        float rudderAngle = -_rudderAngle;
        res.torque.z = res.torque.z < 0 ? Mathf.Min(res.torque.z, rudderAngle * _torqueFactor) :
                       res.torque.z > 0 ? Mathf.Max(res.torque.z, rudderAngle * _torqueFactor) : rudderAngle * _torqueFactor;

        //Debug.Log("AoA: " + aoa + " apparent: " + bodyApparentWater + " direction " + _rudderDirection);
        //Debug.Log(_rudderAngle + " " + res.torque);

        if (_isDebug)
        {
            Vector3 unityDirection = (Quaternion.Euler(0, _rudderAngle, 0) * MathTools.QuaternionNEDtoUnity(_state.rotation)) * Vector3.back;
            Debug.DrawLine(appliPointUnity, appliPointUnity + unityDirection.normalized * 3, Color.magenta);
        }

        //res.force.x = 35000;
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
        //0.5 * _A * v * Mathf.Cos(theta)
        force = ComputeRudderForce();
    }
}
