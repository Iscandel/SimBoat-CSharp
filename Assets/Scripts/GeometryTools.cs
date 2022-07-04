using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

public class GeometryTools
{
    Vector3 TriangleCentroid(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        return (p1 + p2 + p3) / 3;
    }
}

