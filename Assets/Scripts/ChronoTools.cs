using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChronoTools
{ 
    public static ChVectorD Vector3ToChVectorD(Vector3 vec)
    {
        return new ChVectorD(vec.x, vec.y, vec.z);
    }

    public static Vector3 ChVectorDToVector3(ChVectorD vec)
    {
        return new Vector3((float)vec.x, (float)vec.y, (float)vec.z);
    }

    public static ChQuaternionD QuatToChQuat(Quaternion input)
    {
        return new ChQuaternionD(input.w, input.x, input.y, input.z);
    }

    public static Quaternion ChQuatToQuat(ChQuaternionD input)
    {
        return new Quaternion((float)input.e1, (float)input.e2, (float)input.e3, (float)input.e0);
    }
}
