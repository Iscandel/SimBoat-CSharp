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

        _liftDrag = new LiftDrag();
        _liftDrag.LiftCurve = _liftCurve;
        _liftDrag.DragCurve = _dragCurve;
        _liftDrag.Area = _area;
        _liftDrag.Scale = _scale;
        _liftDrag.AppliPoint = _rudderAppliPoint;
        _liftDrag.IsDebug = _isDebug;

        //
        _liftDrag2 = new LiftDrag();
        _liftDrag2.LiftCurve = _liftCurveFxFy;
        _liftDrag2.DragCurve = _dragCurveFxFy;
        _liftDrag2.Area = _area;
        _liftDrag2.Scale = _scale;
        _liftDrag2.AppliPoint = _rudderAppliPoint;
        _liftDrag2.IsDebug = _isDebug;
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

        Vector3 fluidVector_body = Vector3.zero;

        // TODO Remove
        fluidVector_body = MathTools.VectorUnityToNED(Quaternion.Euler(0, _heading, 0) * Vector3.forward);
        Vector3 pos = MathTools.NEDToUnity(_state.worldCenterOfMass);
        DrawArrow.ForDebug(pos, MathTools.VectorNEDToUnity(_state.rotation * fluidVector_body.normalized), Color.yellow);
        //

        res = _liftDrag.ComputeForce(_state, fluidVector_body, _rudderDirection);

        //_liftDrag2.ComputeForce2(_state, fluidVector_body, _rudderDirection);


        // Legacy
        ////var apparentWater = MathTools.VectorUnityToNED(-(Quaternion.Euler(0, _heading, 0) * Quaternion.Euler(0, _rudderAngle, 0) * Vector3.forward));

        ////Vector3 velocityAtRudder = MathTools.VelocityAt(_state.velocity_body, _state.angularVelocity_body, _rudderAppliPoint - Quaternion.Inverse(_state.rotation) * _state.worldCenterOfMass

        //// NED velocity at rudder in local coords
        //Vector3 velocityAtRudder = MathTools.VelocityAt(_state.velocity_body, _state.angularVelocity_body, _rudderAppliPoint);
        //var bodyApparentWater = ComputeApparentWater(Vector3.zero, velocityAtRudder);// _state.velocity_body);

        //// Simplify to only take into account forward speed
        ////bodyApparentWater.y = bodyApparentWater.z= 0;
        //bodyApparentWater.y = 0;     

        //Vector3 dragDir = bodyApparentWater;

        ////apparentWater.x = -5;

        //var liftDir = calculateLiftDirection(bodyApparentWater, new Vector3(1,0,0));
        //float aoa = ComputeAoA(bodyApparentWater);
        //float drag = GetDragCoeff(aoa);
        //float lift = GetLiftCoeff(aoa);
        //float v = _state.velocity.magnitude;
        //float rho = 1026;
        //Vector3 force = 0.5f * rho * _area * v * v * (drag * dragDir.normalized + lift * liftDir.normalized);
        //res.torque = MathTools.Torque(_rudderAppliPoint, force) * _scale;

        // Ensure minimum torque when turning
        //float rudderAngle = -_rudderAngle;
        //res.torque.z = res.torque.z < 0 ? Mathf.Min(res.torque.z, rudderAngle * _torqueFactor) :
        //               res.torque.z > 0 ? Mathf.Max(res.torque.z, rudderAngle * _torqueFactor) : rudderAngle * _torqueFactor;

        //Debug.Log("AoA: " + aoa + " apparent: " + bodyApparentWater + " direction " + _rudderDirection);
        //Debug.Log(_rudderAngle + " " + res.torque);

        if (_isDebug)
        {
            Vector3 unityDirection = (Quaternion.Euler(0, _rudderAngle, 0) * MathTools.QuaternionNEDtoUnity(_state.rotation)) * Vector3.back;
            Debug.DrawLine(appliPointUnity, appliPointUnity + unityDirection.normalized * 3, Color.magenta);
        }

        //// Method 2
        //float betaR = _rudderAngle * Mathf.Deg2Rad;
        //res.force.x = 0.5f * rho * _area * v * v * (lift * Mathf.Sin(betaR) + drag * Mathf.Cos(betaR));
        //res.force.y = 0.5f * rho * _area * v * v * (lift * Mathf.Cos(betaR) - drag * Mathf.Sin(betaR));
        //res.torque = MathTools.Torque(_rudderAppliPoint, force) * _scale;
        ////if (appliPoint.y + _heightOffset < waterHeight)
        ////{
        ////    if (_rudderAngle > 0)
        ////    {
        ////        res.torque = Vector3.Cross(transform.TransformPoint(_rudderAppliPoint) - MathTools.NEDToUnity(_state.worldCenterOfMass), transform.right * 25000 * _state.velocity_body.magnitude);
        ////        res.torque.x = res.torque.z = 0f;
        ////    }
        ////    else if(_rudderAngle < 0)
        ////    {
        ////        res.torque = Vector3.Cross(transform.TransformPoint(_rudderAppliPoint) - MathTools.NEDToUnity(_state.worldCenterOfMass), -transform.right * 25000 * _state.velocity_body.magnitude);
        ////        res.torque.x = res.torque.z = 0f;
        ////    }

        ////    res.torque = MathTools.AngularVectorUnityToNED(res.torque);
        ////    Debug.Log(res.torque);
        //}

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

class LiftDrag
{
    public LiftDrag()
    {
        _appliPoint = Vector3.zero;
        _area = 0;
        _scale = 0;
        _dragCurve = null;
        _liftCurve = null;
        _isDebug = false;
    }

    private Vector3 ComputeApparentWater(Vector3 currentBody, Vector3 velocityBody)
    {
        return currentBody - velocityBody;
    }

    private float ComputeAoA(Vector3 apparentFluid, Vector3 foilDirection)
    {
        return Mathf.Abs(Vector3.Angle(apparentFluid.normalized, foilDirection.normalized));
    }

    public float GetCoeff(float angle, AnimationCurve curve)
    {
        return curve.Evaluate(angle);
    }

    public float GetDragCoeff(float angleOfAttack)
    {
        return GetCoeff(angleOfAttack, _dragCurve);
    }

    public float GetLiftCoeff(float angleOfAttack)
    {
        return GetCoeff(angleOfAttack, _liftCurve);
    }
    
    /// <summary>
    /// foilDirection in body ned
    /// </summary>
    /// <param name="state"></param>
    /// <param name="foilDirection"></param>
    public ForceTorque ComputeForce(BodyState state, Vector3 fluidVector_body, Vector3 foilDirection_body)
    {
        ForceTorque res = new ForceTorque();

        Vector3 velocityAtRudder = MathTools.VelocityAt_NED(state.velocity_body, state.angularVelocity_body, _appliPoint);
        var bodyApparentFluid = ComputeApparentWater(fluidVector_body, velocityAtRudder);// _state.velocity_body);

        // Simplify to only take into account forward speed
        bodyApparentFluid.z = 0;

        Vector3 dragDir = bodyApparentFluid;

        //apparentWater.x = -5;

        Vector3 liftDir = ComputeLiftDirection(bodyApparentFluid, -foilDirection_body);
        float aoa = ComputeAoA(bodyApparentFluid, -foilDirection_body); // All going backward
        float drag = GetDragCoeff(aoa);
        float lift = GetLiftCoeff(aoa);
        float v = 2;// velocityAtRudder.magnitude;//2; 
        float rho = 1026;
        Vector3 force = 0.5f * rho * _area * v * v * (drag * dragDir.normalized + lift * liftDir.normalized);
        res.force = force;
        res.torque = MathTools.TorqueNEDToNED(_appliPoint, force) * _scale;

        if (_isDebug)
            DrawDebug(state, dragDir, liftDir, force, res.torque, bodyApparentFluid, aoa);

        return new ForceTorque();
    }

    public ForceTorque ComputeForce2(BodyState state, Vector3 fluidVector_body, Vector3 foilDirection_body)
    {
        ForceTorque res = new ForceTorque();

        Vector3 velocityAtRudder = MathTools.VelocityAt_NED(state.velocity_body, state.angularVelocity_body, _appliPoint);
        var bodyApparentFluid = ComputeApparentWater(fluidVector_body, velocityAtRudder);// _state.velocity_body);

        // Simplify to only take into account forward speed
        bodyApparentFluid.z = 0;

       // Vector3 liftDir = ComputeLiftDirection(bodyApparentWater);
        float aoa = ComputeAoA(bodyApparentFluid, -foilDirection_body);
        float drag = GetDragCoeff(aoa);
        float lift = GetLiftCoeff(aoa);
        float v = 2;// velocityAtRudder.magnitude;//2; 
        float rho = 1026;

        float angleApparent = -Vector3.SignedAngle(bodyApparentFluid, new Vector3(1, 0, 0), new Vector3(0,0,1)) * Mathf.Deg2Rad; // BetaR
        Debug.Log("Angle app" + angleApparent + " // " + bodyApparentFluid);

        // Method 2
        res.force.x = 0.5f * rho * _area * v * v * (lift * Mathf.Sin(angleApparent) + drag * Mathf.Cos(angleApparent));
        res.force.y = 0.5f * rho * _area * v * v * (lift * Mathf.Cos(angleApparent) - drag * Mathf.Sin(angleApparent));
        res.torque = MathTools.TorqueNEDToNED(_appliPoint, res.force) * _scale;

        if (_isDebug)
            DrawDebug2(state, res.force);//, liftDir, force);

        return res;
    }

    Vector3 ComputeLiftDirection(Vector3 apparentFluid, Vector3 oppositeFoilDirection_body)
    {
        // /!\ Unity angle convention, but returns an appropriate NED vector
        float angle = Vector3.SignedAngle(-apparentFluid, oppositeFoilDirection_body, new Vector3(0, 0, 1));
        float rotAngle = 0;
        if (Mathf.Abs(angle) < 180)
            rotAngle = -90 * Mathf.Sign(angle); // Negative rotation for positive "angle"
        else
            rotAngle = 90 * Mathf.Sign(angle);

        return (Quaternion.AngleAxis(rotAngle, new Vector3(0, 0, 1)) * apparentFluid).normalized;
    }

    public void DrawDebug(BodyState state, Vector3 dragDir, Vector3 liftDir, Vector3 force, Vector3 torque, Vector3 apparentFluid, float aoa)
    {
        Vector3 appliPointUnity = MathTools.NEDToUnity(state.worldCenterOfMass + state.rotation * _appliPoint);

        DrawArrow.ForDebug(appliPointUnity, MathTools.VectorNEDToUnity(state.rotation * dragDir.normalized), Color.green);
        DrawArrow.ForDebug(appliPointUnity, MathTools.VectorNEDToUnity(state.rotation * liftDir.normalized), Color.red);
        DrawArrow.ForDebug(appliPointUnity, MathTools.VectorNEDToUnity(state.rotation * force.normalized), Color.blue);

        Debug.Log("===============================================");
        Debug.Log("Apparent fluid vector: " + apparentFluid);
        Debug.Log("AoA: " + aoa);
        Debug.Log("Drag (normalized) vector: " + dragDir.normalized);
        Debug.Log("Lift (normalized) vector: " + liftDir.normalized);
        Debug.Log("Force vector: " + force);
        Debug.Log("Torque vector: " + torque);
    }

    public void DrawDebug2(BodyState state, /*Vector3 dragDir, Vector3 liftDir,*/ Vector3 force)
    {
        Vector3 appliPointUnity = MathTools.NEDToUnity(state.worldCenterOfMass + state.rotation * _appliPoint);

        DrawArrow.ForDebug(appliPointUnity, MathTools.VectorNEDToUnity(state.rotation * force.normalized), Color.black);
    }

    //if (appliPoint.y + _heightOffset < waterHeight)
    //{
    //    if (_rudderAngle > 0)
    //    {
    //        res.torque = Vector3.Cross(transform.TransformPoint(_rudderAppliPoint) - MathTools.NEDToUnity(_state.worldCenterOfMass), transform.right * 25000 * _state.velocity_body.magnitude);
    //        res.torque.x = res.torque.z = 0f;
    //    }
    //    else if(_rudderAngle < 0)
    //    {
    //        res.torque = Vector3.Cross(transform.TransformPoint(_rudderAppliPoint) - MathTools.NEDToUnity(_state.worldCenterOfMass), -transform.right * 25000 * _state.velocity_body.magnitude);
    //        res.torque.x = res.torque.z = 0f;
    //    }

    //    res.torque = MathTools.AngularVectorUnityToNED(res.torque);
    //    Debug.Log(res.torque);
    //}

    private Vector3 _appliPoint;
    private float _area;
    private float _scale;
    private AnimationCurve _dragCurve;
    private AnimationCurve _liftCurve;
    private bool _isDebug;

    public float Scale { get => _scale; set => _scale = value; }
    public float Area { get => _area; set => _area = value; }
    public Vector3 AppliPoint { get => _appliPoint; set => _appliPoint = value; }
    public AnimationCurve DragCurve { get => _dragCurve; set => _dragCurve = value; }
    public AnimationCurve LiftCurve { get => _liftCurve; set => _liftCurve = value; }
    public bool IsDebug { get => _isDebug; set => _isDebug = value; }
}
