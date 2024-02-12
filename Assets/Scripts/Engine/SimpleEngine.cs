using Assets.Scripts.Physics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimpleEngine : MonoBehaviour, IForceListener, IPhysicsListener
{
    private float _propellerRPM;
    private float _rudderAngle;

    public float _maxPropellerRPM;
    public float _maxRudderAngle;
    public Vector3 _thrustAppliPoint;
    public Vector3 _localThrustDirection;
    public float _heightOffset;
    public bool generateTorque;

    public bool _simplifiedHeightDetection;

    private IWaterProvider _waterProvider;
    private IPhysicsManager _physicsManager;

    private IBody _body;
    private BodyState _state;

    public bool _isDebug;


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
    }

    // Update is called once per frame
    void Update()
    {
        float thrust = 0;

        if (Input.GetKey(KeyCode.Z))
            thrust = 1f;
        else thrust = Input.GetAxis("Accelerate");


        _propellerRPM = thrust * _maxPropellerRPM;

        //
        float angle = 0;
        if (Input.GetKey(KeyCode.D))
            angle = -1;
        else if (Input.GetKey(KeyCode.Q))
            angle = 1;
        else
            angle = -Input.GetAxis("Horizontal");

        _rudderAngle = angle * _maxRudderAngle;
    }

    ForceTorque ComputeThrustForce()
    {
        Vector3 appliPoint = _state.worldCenterOfMass + _state.rotation * _thrustAppliPoint;
        Vector3 thrustDirection = _localThrustDirection;

        Quaternion rotRudder = Quaternion.Euler(0, _rudderAngle, 0);
        thrustDirection = rotRudder * thrustDirection;
        Vector3 worldThrustDirection = _state.rotation * thrustDirection;

        if (_isDebug)
        {
            Vector3 unityAppliPoint = MathTools.NEDToUnity(appliPoint.x, appliPoint.y, appliPoint.z);
            Vector3 unityDirection = MathTools.VectorNEDToUnity(worldThrustDirection);
            Debug.DrawLine(unityAppliPoint, unityAppliPoint + unityDirection.normalized * 3, Color.magenta);
        }

        //float RHO = worldThrustPosition.y < _waterHeightPropellerPos ? UnityPhysicsConstants.RHO : UnityPhysicsConstants.RHO_AIR;
        float thrust = _propellerRPM;
        float waterHeight;
        if (_simplifiedHeightDetection) 
        {
            waterHeight = 0f;
        }
        else 
        {
            float[] heights = new float[1];
            Vector3[] samples = new Vector3[1];
            samples[0] = _thrustAppliPoint;
            if(_waterProvider.SampleHeightAt(samples, ref heights))
                waterHeight = heights[0];
            else
                waterHeight = 0;

        }
        Matrix4x4 mat = Matrix4x4.TRS(_state.position, _state.rotation, new Vector3(1, 1, 1));

        //if (transform.TransformPoint(_thrustAppliPoint).y + _heightOffset > waterHeight)
        if ( mat.MultiplyPoint3x4(_thrustAppliPoint).z + _heightOffset > waterHeight)
            thrust = 0;

        Vector3 thrustForce = worldThrustDirection * thrust;

        ForceTorque force;
        force.force = Quaternion.Inverse(_state.rotation) * thrustForce;
        force.torque = Quaternion.Inverse(_state.rotation) * Vector3.Cross(_state.rotation * _thrustAppliPoint, thrustForce);

        return force;
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
            _body = (IBody) data;
            _physicsManager.AddForceListener(this, _body, RefFrame.BODY_NED);
        }
    }

    //private void FixedUpdate()
    //{
    //    Force force = ComputeThrustForce();
    //    Debug.Log(force.force);
    //    _rigidbody.AddForceAtPosition(force.force, force.appliPoint);
    //}

    public void ComputeForce(IBody body, ref ForceTorque force, BodyState state)
    {
        force = ComputeThrustForce();
    }
}
