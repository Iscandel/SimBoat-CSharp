using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class YachtMastController : MonoBehaviour
{
    public GameObject _mast;
    public float _rotationSpeed; // degrees / s
    public Vector3 _pivot;

    private IDynamics _actuatorDynamics;
    private float _currentAngle;
    private float _lastAngle;

    private HingeJoint _joint;

    // Start is called before the first frame update
    void Start()
    {
        _actuatorDynamics = new LinearActuatorDynamics(_rotationSpeed);
        _currentAngle = _lastAngle = 0;
        _joint = GetComponentInChildren<HingeJoint>();
    }

    // Update is called once per frame
    void Update()
    {
        float setpoint = 0;
        float moving = 0;
        if (Input.GetKey(KeyCode.A))
            moving = -1;
        else if (Input.GetKey(KeyCode.E))
            moving = 1;

        setpoint = _currentAngle + moving * _rotationSpeed * Time.deltaTime;

        _actuatorDynamics.SetSetpoint(setpoint);  
    }

    public void SetSetpoint(float setpoint)
    {
        _actuatorDynamics.SetSetpoint(setpoint);
    }

    private void FixedUpdate()
    {
        _actuatorDynamics.Update(Time.fixedDeltaTime);
        _currentAngle = _actuatorDynamics.GetState();
        
        var rigidbody = GetComponent<Rigidbody>();
        _joint.connectedBody = rigidbody;

        Vector3 worldPoint = transform.position + transform.rotation * _pivot;
        JointLimits jointLimits = new JointLimits();
        jointLimits.min = _currentAngle - 0.1f;
        jointLimits.max = _currentAngle + 0.1f;
        
        _joint.limits = jointLimits;
        //_mast.transform.RotateAround(worldPoint, Vector3.up, _currentAngle - _lastAngle);

        _lastAngle = _currentAngle;
    }
}
