using JetBrains.Annotations;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

class LinearActuatorDynamics : IDynamics
{
    private float _absSpeed;
    private float _setpoint;
    private float _state;
    private bool _done;

    public LinearActuatorDynamics(float absSpeed)
    {
        _absSpeed = absSpeed;
        _setpoint = 0;
        _state = 0;
        _done = true;
    }

    public LinearActuatorDynamics(float absSpeed, float setpoint, float state)
    {
        _absSpeed = absSpeed;
        _setpoint = setpoint;
        _state = state;
        _done = _setpoint != _state;
    }

    public float GetSetpoint()
    {
        return _setpoint;
    }

    public float GetState()
    {
        return _state;
    }

    public void SetSetpoint(float value) 
    {
        if(value != _setpoint)
        {
            _setpoint = value;
            _done = false;
        }
    }

    public void SetState(float value)
    {
        if (value != _state)
        {
            _state = value;
            _done = _setpoint != _state;
        }
    }

    public void Update(float dt)
    {
        if (_done)
            return;

        float speed = _absSpeed;
        bool wasNeg = false;
        if (_setpoint - _state < 0)
        {
            wasNeg = true;
            speed = -_absSpeed;
        }


        _state += speed * dt;
        if ((_setpoint - _state < 0) != wasNeg)
        {
            _state = _setpoint;
            _done = true;
        }
    }

}
