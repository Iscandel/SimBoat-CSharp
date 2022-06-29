using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;


internal class SimpleWaterProvider : IWaterProvider
{
    private float _waterLevel;

    public SimpleWaterProvider(float height)
    {
        _waterLevel = height;
    }

    public float GetHeightAt(Vector3 samplePoint)
    {
        return _waterLevel;
    }

    public float GetHeightAt(int listIndex)
    {
        return _waterLevel;
    }

    public bool UpdateSamplingList(Vector3[] samplePoints)
    {
        return true;
    }
}
