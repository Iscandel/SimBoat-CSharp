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

    // Start is called before the first frame update
    void Start()
    {
        _actuatorDynamics = new LinearActuatorDynamics(_rotationSpeed);
        _currentAngle = _lastAngle = 0;
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

    private void FixedUpdate()
    {
        _actuatorDynamics.Update(Time.fixedDeltaTime);
        _currentAngle = _actuatorDynamics.GetState();

        Vector3 worldPoint = transform.position + transform.rotation * _pivot;

        _mast.transform.RotateAround(worldPoint, Vector3.up, _currentAngle - _lastAngle);

        _lastAngle = _currentAngle;
    }
}
