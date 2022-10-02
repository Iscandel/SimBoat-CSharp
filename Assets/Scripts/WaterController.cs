using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

class WaterController
{
    public static float _waveVelocity = 1f;
    public static float _wavelength = 5f;
    public static float _waterHeight = 0.0f;

    public static float GetPosY(Vector3 pos, float time )
    {
        return Mathf.Sin((-_waveVelocity * time + pos.x) / _wavelength) * _waterHeight + Mathf.Sin((-_waveVelocity * 1.5f * time + pos.x) / _wavelength / 3) * _waterHeight + Mathf.Sin((-_waveVelocity / 4.0f * time + pos.x) / _wavelength / 2) * _waterHeight * 1.3f; ;
    }
}
