using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;


public class SimpleWaterProvider : IWaterProvider
{
    private float _waterLevel;

    public SimpleWaterProvider(float height)
    {
        _waterLevel = height;
    }

    public override void SetSamplingTime(float time) { }

    public override bool SampleHeightAt(Vector3[] samplePoints, ref float[] heights)
    {
        for(int i = 0; i < samplePoints.Length; i++) 
        {
            heights[i] = _waterLevel;
        }

        return true;
    }

    //public float GetHeightAt(int listIndex)
    //{
    //    return _waterLevel;
    //}

    //public bool UpdateSamplingList(Vector3[] samplePoints)
    //{
    //    return true;
    //}
}
