using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HydrodynamicsComplex : MonoBehaviour
{
    float _surface = 0;
    BoatForcesUnity _boatForcesUnity;
    Rigidbody _rigidbody;

    public float CX = 1.05f * 10;
    public float CXuv = 1.05f * 10;
    public float CXur = 1.05f * 10;
    public float CY = 1.5f * 10;
    public float CZ = 0.5f * 10;
    public float CL = 0;
    public float CM = 0;
    public float CN = 1;

    // Start is called before the first frame update
    void Start()
    {
        _boatForcesUnity = GetComponent<BoatForcesUnity>();
        _rigidbody = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {

    }

    void FixedUpdate()
    {
        (Vector3 damping2, Vector3 torque) = ComputeDamping(_rigidbody.velocity, _rigidbody.angularVelocity);
        _rigidbody.AddForce(damping2);
        _rigidbody.AddTorque(torque);
    }

    public (Vector3, Vector3) ComputeDamping(Vector3 speed, Vector3 angularSpeed)
    {
        _surface = _boatForcesUnity.Surface;
        Debug.Log("surf " + _surface);
        Vector3 speed_body = transform.InverseTransformDirection(speed);
        Vector3 angularSpeed_body = transform.InverseTransformDirection(angularSpeed);

        //Debug.Log("SPEED " + speed + " " + speed_body);


        Vector3 force;
        float RHO = UnityPhysicsConstants.RHO;
        float referenceArea = _surface;// _submergedSurface / 4;
        float halfRho = 0.5f * RHO * referenceArea;
        //float Cdx = 1.05f * 10;
        //float Cdy = 1.5f * 10;
        //float Cdz = 0.5f * 10;
        //float Cax = 0;
        //float Cay = 0;
        //float Caz = 1;
        force.x = -halfRho * (CX * speed_body.x + CXuv * speed_body.x * speed_body.z + CXur * speed_body.z * angularSpeed_body.y);
        force.y = -halfRho * CY  * speed_body.y;
        force.z = -halfRho * CZ /** Mathf.Abs(speed_body.z)*/ * speed_body.z;

        Vector3 torque;
        torque.x = -0.5f * RHO * referenceArea * CL /** Mathf.Abs(angularSpeed.x)*/ * angularSpeed_body.x;
        torque.y = -0.5f * RHO * referenceArea * CM /** Mathf.Abs(angularSpeed.y)*/ * angularSpeed_body.y;
        torque.z = -0.5f * RHO * referenceArea * CN /** Mathf.Abs(angularSpeed.z)*/ * angularSpeed_body.z;

        //Debug.Log("HYDRO " + force + " " + torque);
        return (transform.TransformDirection(force), transform.TransformDirection(torque));
    }
}
