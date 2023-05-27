using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CoMViewer : MonoBehaviour
{
    public float _radius = 0;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnDrawGizmos()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        if(rb)
        {
            // Draw a yellow sphere at the transform's position
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(rb.worldCenterOfMass, _radius);
        }
        
    }
}
