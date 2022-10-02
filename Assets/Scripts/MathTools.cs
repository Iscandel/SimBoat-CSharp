using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

class MathTools
{
    public const float RPM_TO_RPS = 1.0f / 60;
    public static Vector3 UnityToNED(float x, float y, float z)
    {
        return new Vector3(z, x, -y);
    }

    public static Vector3 NEDToUnity(float x, float y, float z)
    {
        return new Vector3(y, -z, x);
    }

    public static float Interp1(float start, float end, float t)
    {
        return (1 - t) * start + t * end;
    }
}