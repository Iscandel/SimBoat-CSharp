using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

public class GeometryTools
{
    public static Vector3 MidPoint(Vector3 a, Vector3 b)
    {
        return (a + b) / 2.0f;
    }

    public static Vector3 ComputeCentroid(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        return (p1 + p2 + p3) / 3;
    }

    public static Vector3 ComputeBarycenter(float alpha, Vector3 A, float beta, Vector3 B)
    {
        if (alpha + beta == 0)
            return new Vector3(0, 0, 0);

        Vector3 AB = B - A;
        return A + beta / (alpha + beta) * AB;
    }

    public static float ComputeTriangleArea(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        Vector3 normal = Vector3.Cross(p2 - p1, p3 - p1);
        float area = 0.5f * normal.magnitude;

        return area;
    }

    /// <summary>
    /// Returns the (normalized) triangle normal
    /// </summary>
    /// <param name="p0"></param>
    /// <param name="p1"></param>
    /// <param name="p2"></param>
    /// <returns></returns>
    public static Vector3 ComputeNormal(Vector3 p0, Vector3 p1, Vector3 p2)
    {
        Vector3 normal = Vector3.Cross(p1 - p0, p2 - p0);
        return normal.normalized;
    }
}

