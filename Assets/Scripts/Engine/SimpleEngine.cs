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

    public bool generateTorque;


    // Start is called before the first frame update
    void Start()
    {
        _rigidbody = GetComponent<Rigidbody>();
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
        Vector3 thrustForce = worldThrustDirection * thrust;

        Force force;
        force.force = thrustForce;
        force.appliPoint = appliPoint;

        return force;
    }

    private void FixedUpdate()
    {
        Force force = ComputeThrustForce();
        _rigidbody.AddForceAtPosition(force.force, force.appliPoint);
    }
}
