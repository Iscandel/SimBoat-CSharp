using System.Collections;
using System.Collections.Generic;
using UnityEngine;
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

    private Vector3 ComputeApparentWater(Vector3 fluidVector_body, Vector3 velocityVector_body)
    {
        return fluidVector_body - velocityVector_body;
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
    public ForceTorque ComputeForce(BodyState state, Vector3 fluidVector_body, Vector3 foilDirection_body, float rho)
    {
        ForceTorque res = new ForceTorque();

        Vector3 velocityAtAppliPoint = MathTools.VelocityAt_NEDToNED(state.velocity_body, state.angularVelocity_body, _appliPoint);
        var apparentFluid_body = ComputeApparentWater(fluidVector_body, velocityAtAppliPoint);// _state.velocity_body);

        // Simplify to only take into account forward speed
        apparentFluid_body.z = 0;

        Vector3 dragDir = apparentFluid_body;

        //apparentWater.x = -5;

        Vector3 liftDir = ComputeLiftDirection(apparentFluid_body, -foilDirection_body);
        float aoa = ComputeAoA(apparentFluid_body, -foilDirection_body); // All going backward
        float drag = GetDragCoeff(aoa);
        float lift = GetLiftCoeff(aoa);
        float v = apparentFluid_body.magnitude;//2; 
        Vector3 force = 0.5f * rho * _area * v * v * (drag * dragDir.normalized + lift * liftDir.normalized);
        res.force = force;
        res.torque = MathTools.TorqueNEDToNED(_appliPoint, force) * _scale;

        if (_isDebug)
            DrawDebug(state, dragDir, liftDir, force, res.torque, apparentFluid_body, aoa);

        return res;
    }

    public ForceTorque ComputeForce2(BodyState state, Vector3 fluidVector_body, Vector3 foilDirection_body)
    {
        ForceTorque res = new ForceTorque();

        Vector3 velocityAtRudder = MathTools.VelocityAt_NEDToNED(state.velocity_body, state.angularVelocity_body, _appliPoint);
        var bodyApparentFluid = ComputeApparentWater(fluidVector_body, velocityAtRudder);// _state.velocity_body);

        // Simplify to only take into account forward speed
        bodyApparentFluid.z = 0;

        // Vector3 liftDir = ComputeLiftDirection(bodyApparentWater);
        float aoa = ComputeAoA(bodyApparentFluid, -foilDirection_body);
        float drag = GetDragCoeff(aoa);
        float lift = GetLiftCoeff(aoa);
        float v = 2;// velocityAtRudder.magnitude;//2; 
        float rho = 1026;

        float angleApparent = -Vector3.SignedAngle(bodyApparentFluid, new Vector3(1, 0, 0), new Vector3(0, 0, 1)) * Mathf.Deg2Rad; // BetaR
        //Debug.Log("Angle app" + angleApparent + " // " + bodyApparentFluid);

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
