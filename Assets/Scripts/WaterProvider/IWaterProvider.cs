using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

//interface IWaterProvider
public abstract class IWaterProvider : MonoBehaviour
{
    public abstract void SetSamplingTime(float time);
    // bool UpdateSamplingList(Vector3[] samplePoints);

    public abstract bool SampleHeightAt(Vector3[] samplePoints, ref float[] height);// int listIndex);
    //float GetHeightAt(Vector3 samplePoint);
}

