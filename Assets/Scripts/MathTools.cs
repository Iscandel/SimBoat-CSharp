using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using UnityEngine;

public static class MathTools
{
    public const float RPM_TO_RPS = 1.0f / 60;
    public static Vector3 UnityToNED(float x, float y, float z)
    {
        return new Vector3(z, x, -y);
    }

    /// <summary>
    /// out Vector3(xUCS, yUnity, zUnity)
    /// </summary>
    /// <param name="xSCS"></param>
    /// <param name="yNED"></param>
    /// <param name="zNED"></param>
    /// <returns></returns>
    public static Vector3 PositionNEDToUnity(float xNED, float yNED, float zNED)
    {
        return new Vector3((float)yNED, -(float)zNED, (float)xNED);
    }

    /// <summary>
    /// out Vector3(pitchUnity, yawUnity, rollUnity) = rotation around (xUnity, yUnity, zUnity)
    /// </summary>
    /// <param name="pitchNED">rotation around yNED</param>
    /// <param name="yawNED">rotation around zNED</param>
    /// <param name="rollNED">rotation around xNED</param>
    /// <returns></returns>
    public static Vector3 EulerAnglesNEDToUnity(float pitchNED, float yawNED, float rollNED)
    {
        return new Vector3( -pitchNED, yawNED, -rollNED);
    }

    /// <summary>
    /// out Vector3(pitchUnity, yawUnity, rollUnity) = rotation around (xUnity, yUnity, zUnity)
    /// </summary>
    /// <param name="pitchNED">rotation around yNED</param>
    /// <param name="yawNED">rotation around zNED</param>
    /// <param name="rollNED">rotation around xNED</param>
    /// <returns></returns>
    public static Quaternion QuaternionAngleNEDtoUnity(float pitchNED, float yawNED, float rollNED)
    {
        return Quaternion.Euler(-pitchNED, yawNED, -rollNED);
    }
    /// <summary>
    /// Converts a NED vector in Unity frame
    /// </summary>
    /// <param name="xNED"></param>
    /// <param name="yNED"></param>
    /// <param name="zNED"></param>
    /// <returns>Vector3(xUnity, yUnity, zUnity)</returns>
    public static Vector3 VectorNEDToUnity(float xNED, float yNED, float zNED)
    {
        return new Vector3(yNED, -zNED, xNED);
    }
    /// <summary>
    ///  Converts a NED vector in Unity frame
    /// </summary>
    /// <param name="vectorNED"></param>
    /// <returns>Vector3(xUnity, yUnity, zUnity)</returns>
    public static Vector3 VectorNEDToUnity(Vector3 vectorNED)
    {
        return new Vector3(vectorNED.y, -vectorNED.z, vectorNED.x);
    }
    /// <summary>
    /// Converts a NED vector expressing angular quantities (angular velocity, 
    /// angular acceleration) in a Unity one
    /// </summary>
    /// <param name="xNED"></param>
    /// <param name="yNED"></param>
    /// <param name="zNED"></param>
    /// <returns>Vector3(xUnity, yUnity, zUnity)</returns>
    public static Vector3 AngularVectorNEDToUnity(float xNED, float yNED, float zNED)
    {
        return new Vector3(-yNED, zNED, -xNED);
    }
    /// <summary>
    /// Converts a NED vector expressing angular quantities (angular velocity, 
    /// angular acceleration) in a Unity one
    /// </summary>
    /// <param name="vectorNED"></param>
    /// <returns>Vector3(xUnity, yUnity, zUnity)</returns>
    public static Vector3 AngularVectorNEDToUnity(Vector3 vectorNED)
    {
        return new Vector3(-vectorNED.y, vectorNED.z, -vectorNED.x);
    }
    /// <summary>
    /// Converts a NED quaternion in its Unity equivalent
    /// </summary>
    /// <param name="NEDQuat"></param>
    /// <returns></returns>
    public static Quaternion QuaternionNEDtoUnity(Quaternion NEDQuat)
    {
        // Correct I guess
        Vector3 axis = AngularVectorNEDToUnity(NEDQuat.x, NEDQuat.y, NEDQuat.z);
        Quaternion res = new Quaternion(axis.x, axis.y, axis.z, NEDQuat.w);
        return res;
    }

    /// <summary>
    /// out Vector3(xNED, yNED, zNED)
    /// </summary>
    /// <param name="positionUnity">Vector3(xUnity, yUnity, zUnity)</param>
    /// <returns></returns>
    public static Vector3 PositionUnityToNED(Vector3 positionUnity)
    {
        return new Vector3((float)positionUnity.z,  (float)positionUnity.x, -(float)positionUnity.y);
    }

    /// <summary>
    /// out Vector3(xNED, yNED, zNED)
    /// </summary>
    /// <param name="xUnity"></param>
    /// <param name="yUnity"></param>
    /// <param name="zUnity"></param>
    /// <returns></returns>
    public static Vector3 PositionUnityToNED(float xUnity, float yUnity, float zUnity)
    {
        return new Vector3((float)zUnity, (float)xUnity, -(float)yUnity);
    }

    /// <summary>
    /// Converts a Unity vector in NED frame
    /// </summary>
    /// <param name="xUnity"></param>
    /// <param name="yUnity"></param>
    /// <param name="zUnity"></param>
    /// <returns>Vector3(xNED, yNED, zNED)</returns>
    public static Vector3 VectorUnityToNED(float xUnity, float yUnity, float zUnity)
    {
        return new Vector3((float)zUnity, (float)xUnity, -(float)yUnity);
    }
    /// <summary>
    /// Converts a Unity vector in NED frame
    /// </summary>
    /// <param name="vectorUnity"></param>
    /// <returns>Vector3(xNED, yNED, zNED)</returns>
    public static Vector3 VectorUnityToNED(Vector3 vectorUnity)
    {
        return new Vector3((float)vectorUnity.z, (float)vectorUnity.x, -(float)vectorUnity.y);
    }
    /// <summary>
    /// Converts a Unity quaternion in its NED equivalent
    /// </summary>
    /// <param name="UnityQuat"></param>
    /// <returns></returns>
    public static Quaternion QuaternionUnityToNED(Quaternion UnityQuat)
    {
        // Left handed frame ? Has to be negated anyway
        Vector3 axis = -VectorUnityToNED(UnityQuat.x, UnityQuat.y, UnityQuat.z);
        Quaternion res = new Quaternion(axis.x, axis.y, axis.z, UnityQuat.w);
        return res;
    }
    /// <summary>
    /// Converts a Unity vector expressing angular quantities (angular velocity, 
    /// angular acceleration) in a NED one
    /// </summary>
    /// <param name="xUnity"></param>
    /// <param name="yUnity"></param>
    /// <param name="zUnity"></param>
    /// <returns>Vector3(xNED, yNED, zNED)</returns>
    public static Vector3 AngularVectorUnityToNED(float xUnity, float yUnity, float zUnity)
    {
        return new Vector3(-zUnity, -xUnity, yUnity);
    }
    /// <summary>
    /// Converts a Unity vector expressing angular quantities (angular velocity, 
    /// angular acceleration) in a NED one
    /// </summary>
    /// <param name="vectorUnity"></param>
    /// <returns>Vector3(xNED, yNED, zNED)</returns>
    public static Vector3 AngularVectorUnityToNED(Vector3 vectorUnity)
    {
        return new Vector3(-vectorUnity.z, -vectorUnity.x, vectorUnity.y);
    }

    /// <summary>
    /// out Vector3(rollNED, pitchNED, yawNED) = rotation around (xNED, yNED, zNED) 
    /// </summary>
    /// <param name="pitchUnity">rotation around xUnity</param>
    /// <param name="yawUnity">rotation around yUnity</param>
    /// <param name="rollUnity">rotation around zUnity</param>
    /// <returns></returns>
    public static Vector3 EulerAnglesUnitytoNED(float pitchUnity, float yawUnity, float rollUnity)
    {
        return new Vector3(-rollUnity, -pitchUnity, yawUnity);
    }

    /// <summary>
    /// out Vector3(rollNED, pitchNED, yawNED) = rotation around (xNED, yNED, zNED) 
    /// </summary>
    /// <param name="rotationUnity">Vector3(pitchUnity, yawUnity, rollUnity) = rotation around (xUnity, yUnity, zUnity) </param>
    /// <returns></returns>
    public static Vector3 EulerAnglesUnitytoNED(Vector3 rotationUnity)
    {
        return new Vector3(-rotationUnity.z, -rotationUnity.x, rotationUnity.y);
    }

    public static Vector3 NEDToUnity(float x, float y, float z)
    {
        return new Vector3(y, -z, x);
    }

    public static float Interp1(float start, float end, float t)
    {
        return (1 - t) * start + t * end;
    }

    public static bool AlmostEqual(this Vector3 a, Vector3 b, float threshold = 0.005f)
    {
        return Vector3.SqrMagnitude(a - b) < threshold;
    }
}