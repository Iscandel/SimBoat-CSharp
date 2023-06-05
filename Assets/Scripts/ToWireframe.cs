// from http://docs.unity3d.com/ScriptReference/GL-wireframe.html
using UnityEngine;
using System.Collections;

public class ToWireframe : MonoBehaviour
{
    void OnPreRender()
    {
        GL.wireframe = true;
    }
    void OnPostRender()
    {
        GL.wireframe = false;
    }
}
