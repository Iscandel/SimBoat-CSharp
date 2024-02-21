using Assets.Scripts.Physics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HydrodynamicsDampingSimple : MonoBehaviour, IForceListener, IPhysicsListener
{
    float _surface = 0;
    MeshBasedWaterPhysics _boatForcesUnity;

    public float CX = 1.05f * 10;
    public float CX2 = 0;
    public float CYuv = 1.05f * 10;
    public float CYur = 1.05f * 10;
    public float CY = 1.5f * 10;
    public float CZ = 0.5f * 10;
    public float CL = 0;
    public float CM = 0;
    public float CN = 1;

    private IPhysicsManager _physicsManager;

    private IBody _body;
    private BodyState _state;

    private RefFrame _refFrame = RefFrame.BODY_NED;//

    // Start is called before the first frame update
    void Start()
    {
        _boatForcesUnity = GetComponent<MeshBasedWaterPhysics>();

        GameObject[] physicsManager = GameObject.FindGameObjectsWithTag("PhysicsManager");
        _physicsManager = physicsManager[0].GetComponent<IPhysicsManager>();
        _physicsManager.AddPhysicsEventListener(this);
        if (_boatForcesUnity.Body != null)
        {
            _body = _boatForcesUnity.Body;
            _physicsManager.AddForceListener(this, _body, _refFrame);
        }
    }

    // Update is called once per frame
    void Update()
    {

    }

    //void FixedUpdate()
    //{
    //    if (_rigidbody.position.y < 0.8)
    //    {
    //        (Vector3 damping2, Vector3 torque) = ComputeDamping3(_rigidbody.velocity, _rigidbody.angularVelocity);
    //        _rigidbody.AddForce(damping2);
    //        _rigidbody.AddTorque(torque);
    //    }
    //}

    public ForceTorque ComputeDamping()
    {
        ForceTorque force;
        _surface = _boatForcesUnity.MeshArea;


        Vector3 speed_body = _state.velocity_body;
        Vector3 angularSpeed_body = _state.angularVelocity_body;

        float RHO = UnityPhysicsConstants.RHO;
        float referenceArea = _surface;// _submergedSurface / 4;

        force.force.x = -0.5f * RHO * referenceArea * (CX /** Mathf.Abs(speed_body.y)*/ * speed_body.x + CX2 * Mathf.Abs(speed_body.x) * speed_body.x);
        force.force.y = -0.5f * RHO * referenceArea * (CY * Mathf.Abs(speed_body.y) * speed_body.y + CYuv * speed_body.x * speed_body.y + CYur * speed_body.x * angularSpeed_body.z);
        force.force.z = -0.5f * RHO * referenceArea * CZ /** Mathf.Abs(speed_body.z)*/ * speed_body.z;

        force.torque.x = -0.5f * RHO * referenceArea * CL /** Mathf.Abs(angularSpeed.x)*/ * angularSpeed_body.x;
        force.torque.y = -0.5f * RHO * referenceArea * CM /** Mathf.Abs(angularSpeed.y)*/ * angularSpeed_body.y;
        force.torque.z = -0.5f * RHO * referenceArea * CN /** Mathf.Abs(angularSpeed.z)*/ * angularSpeed_body.z;

        //Debug.Log("HYDRO " + force + " " + torque);

        return force;
    }

    public void OnPhysicsEvent(IPhysicsListener.EventType eventType, object data)
    {
        if (eventType == IPhysicsListener.EventType.STATE_UPDATED)
        {
            _state = _physicsManager.GetBodyState(_body, RefFrame.NED);
        }
        else if (eventType == IPhysicsListener.EventType.BODY_CREATED)
        {
            _body = (IBody)data;
            _physicsManager.AddForceListener(this, _body, _refFrame);
        }
    }

    public void ComputeForce(IBody body, ref ForceTorque force, BodyState state)
    {
        force = ComputeDamping();
    }
}





///////////////////////////////////////////////
///






//using Assets.Scripts.Physics;
//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;

//public class HydrodynamicsDampingSimple : MonoBehaviour, IForceListener, IPhysicsListener
//{
//    float _surface = 0;
//    MeshBasedWaterPhysics _boatForcesUnity;

//    //public float CX = 1.05f * 10;
//    //public float CYuv = 1.05f * 10;
//    //public float CYur = 1.05f * 10;
//    //public float CY = 1.5f * 10;
//    //public float CZ = 0.5f * 10;
//    //public float CL = 0;
//    //public float CM = 0;
//    //public float CN = 1;


//    public float CX = 1.05f * 10;
//    public float CXuv = 1.05f * 10;
//    public float CXur = 1.05f * 10;
//    public float CY = 1.5f * 10;
//    public float CZ = 0.5f * 10;
//    public float CL = 0;
//    public float CM = 0;
//    public float CN = 1;

//    private IPhysicsManager _physicsManager;

//    private IBody _body;
//    private BodyState _state;

//    private RefFrame _refFrame = RefFrame.WORLD_UNITY;// BODY_NED;//

//    // Start is called before the first frame update
//    void Start()
//    {
//        _boatForcesUnity = GetComponent<MeshBasedWaterPhysics>();

//        GameObject[] physicsManager = GameObject.FindGameObjectsWithTag("PhysicsManager");
//        _physicsManager = physicsManager[0].GetComponent<IPhysicsManager>();
//        _physicsManager.AddPhysicsEventListener(this);
//        if (_boatForcesUnity.Body != null)
//        {
//            _body = _boatForcesUnity.Body;
//            _physicsManager.AddForceListener(this, _body, _refFrame);
//        }
//    }

//    // Update is called once per frame
//    void Update()
//    {

//    }

//    //void FixedUpdate()
//    //{
//    //    if (_rigidbody.position.y < 0.8)
//    //    {
//    //        (Vector3 damping2, Vector3 torque) = ComputeDamping3(_rigidbody.velocity, _rigidbody.angularVelocity);
//    //        _rigidbody.AddForce(damping2);
//    //        _rigidbody.AddTorque(torque);
//    //    }
//    //}

//    //public ForceTorque ComputeDamping()
//    //{
//    //    ForceTorque force;
//    //    _surface = _boatForcesUnity.MeshArea;


//    //    Vector3 speed_body = _state.velocity_body;
//    //    Vector3 angularSpeed_body = _state.angularVelocity_body;

//    //    float RHO = UnityPhysicsConstants.RHO;
//    //    float referenceArea = _surface;// _submergedSurface / 4;

//    //    force.force.x = -0.5f * RHO * referenceArea * CX /** Mathf.Abs(speed_body.y)*/ * speed_body.x;
//    //    force.force.y = -0.5f * RHO * referenceArea * (CY * Mathf.Abs(speed_body.y) * speed_body.y + CYuv * speed_body.x * speed_body.y + CYur * speed_body.x * angularSpeed_body.z);
//    //    force.force.z = -0.5f * RHO * referenceArea * CZ /** Mathf.Abs(speed_body.z)*/ * speed_body.z;

//    //    force.torque.x = -0.5f * RHO * referenceArea * CL /** Mathf.Abs(angularSpeed.x)*/ * angularSpeed_body.x;
//    //    force.torque.y = -0.5f * RHO * referenceArea * CM /** Mathf.Abs(angularSpeed.y)*/ * angularSpeed_body.y;
//    //    force.torque.z = -0.5f * RHO * referenceArea * CN /** Mathf.Abs(angularSpeed.z)*/ * angularSpeed_body.z;

//    //    //Debug.Log("HYDRO " + force + " " + torque);

//    //    return force;
//    //}

//    public ForceTorque ComputeDamping2()
//    {
//        ForceTorque force;
//        _surface = _boatForcesUnity.MeshArea;


//        Vector3 speed_body = _state.velocity_body;
//        Vector3 angularSpeed_body = _state.angularVelocity_body;

//        float RHO = UnityPhysicsConstants.RHO;
//        float referenceArea = _surface;// _submergedSurface / 4;
//        force.force.x = -0.5f * RHO * referenceArea * (CX * Mathf.Abs(speed_body.x) * speed_body.x + CXuv * speed_body.x * speed_body.z + CXur * speed_body.z * angularSpeed_body.y);
//        force.force.y = -0.5f * RHO * referenceArea * CY /** Mathf.Abs(speed_body.y)*/ * speed_body.y;
//        force.force.z = -0.5f * RHO * referenceArea * CZ /** Mathf.Abs(speed_body.z)*/ * speed_body.z;

//        Vector3 torque;
//        force.torque.x = -0.5f * RHO * referenceArea * CL /** Mathf.Abs(angularSpeed.x)*/ * angularSpeed_body.x;
//        force.torque.y = -0.5f * RHO * referenceArea * CM /** Mathf.Abs(angularSpeed.y)*/ * angularSpeed_body.y;
//        force.torque.z = -0.5f * RHO * referenceArea * CN /** Mathf.Abs(angularSpeed.z)*/ * angularSpeed_body.z;
//        force.force = transform.TransformDirection(force.force);
//        force.torque = transform.TransformDirection(force.torque);
//        //Debug.Log("HYDRO " + force + " " + torque);
//        return force;
//    }

//    public void OnPhysicsEvent(IPhysicsListener.EventType eventType, object data)
//    {
//        if (eventType == IPhysicsListener.EventType.STATE_UPDATED)
//        {
//            _state = _physicsManager.GetBodyState(_body, RefFrame.UNITY);
//        }
//        else if (eventType == IPhysicsListener.EventType.BODY_CREATED)
//        {
//            _body = (IBody)data;
//            _physicsManager.AddForceListener(this, _body, _refFrame);
//        }
//    }

//    public void ComputeForce(IBody body, ref ForceTorque force, BodyState state)
//    {
//        force = ComputeDamping2();
//    }
//}