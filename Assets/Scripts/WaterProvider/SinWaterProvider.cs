using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;
class SinWaterProvider : IWaterProvider
{
    float[] _samplingList;
    static float _samplingTime;

    public SinWaterProvider()
    {
        _samplingTime = 0;
    }

    public void SetSamplingTime(float time) 
    {
        _samplingTime = time;
    }

    //public float GetHeightAt(Vector3 samplePoint)
    //{
    //    return WaterController.GetPosY(samplePoint, _samplingTime);
    //}

    //public float GetHeightAt(int listIndex)
    //{
    //    return _samplingList[listIndex];
    //}

    public bool SampleHeightAt(Vector3[] samplePoints, ref float[] heights)
    {
        if(samplePoints.Length == 0)
            return false;

        _samplingList = new float[samplePoints.Length];
        if(heights.Length < samplePoints.Length)
        {
            heights = new float[samplePoints.Length];   
        }

        int i = 0;
        foreach(Vector3 samplePoint in samplePoints)
        {
            heights[i++] = WaterController.GetPosY(samplePoint, _samplingTime);
        }

        return true;
    }
}
