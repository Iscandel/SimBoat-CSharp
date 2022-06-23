using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

interface IWaterProvider
{
    float GetHeightAt(int listIndex);
    float GetHeightAt(Vector3 samplePoint);
}

