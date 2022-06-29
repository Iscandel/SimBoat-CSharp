using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

class MathTools
{
    Vector3 UnityToNED(float x, float y, float z)
    {
        return new Vector3(z, x, -y);
    }

    Vector3 NEDToUnity(float x, float y, float z)
    {
        return new Vector3(y, -z, x);
    }
}