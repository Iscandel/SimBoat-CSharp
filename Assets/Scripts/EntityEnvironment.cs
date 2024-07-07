using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EntityEnvironment : MonoBehaviour
{
    public Vector3 _currentVector;
    public float _rho;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public Vector3 GetCurrentVector_NED()
    {
        return _currentVector;
    }

    public float GetRho()
    {
        return _rho;
    }
}
