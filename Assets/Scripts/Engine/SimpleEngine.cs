using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimpleEngine : MonoBehaviour
{
    private Rigidbody _rigidbody;
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


    // Start is called before the first frame update
    void Start()
    {
        _rigidbody = GetComponent<Rigidbody>();
        _simplifiedHeightDetection = true;
        //_heightOffset = 0;

        GameObject[] oceanGO = GameObject.FindGameObjectsWithTag("Ocean");
        _waterProvider = oceanGO[0].GetComponent<IWaterProvider>();
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

    Force ComputeThrustForce()
    {
        Vector3 appliPoint = _rigidbody.worldCenterOfMass + transform.TransformDirection(_thrustAppliPoint);
        Vector3 thrustDirection = _localThrustDirection;

        Quaternion rotRudder = Quaternion.Euler(0, _rudderAngle, 0);
        thrustDirection = rotRudder * thrustDirection;
        Vector3 worldThrustDirection = transform.rotation * thrustDirection;

        Debug.DrawLine(appliPoint, appliPoint + worldThrustDirection.normalized * 3, Color.magenta);

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

        if (transform.TransformPoint(_thrustAppliPoint).y + _heightOffset > waterHeight)
            thrust = 0;

        Vector3 thrustForce = worldThrustDirection * thrust;

        Force force;
        force.force = thrustForce;
        force.appliPoint = appliPoint;

        return force;
    }

    private void FixedUpdate()
    {
        Force force = ComputeThrustForce();
        Debug.Log(force.force);
        _rigidbody.AddForceAtPosition(force.force, force.appliPoint);
    }
}
