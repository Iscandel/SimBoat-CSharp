using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine.XR;

interface IDynamics
{
    public void SetSetpoint(float value);
    public float GetSetpoint();
    public void SetState(float value);
    public float GetState();
    public void Update(float dt);
}
