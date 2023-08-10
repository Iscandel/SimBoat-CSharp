using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BodyState
{
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 velocity;
    public Vector3 velocity_body;
    public Vector3 angularVelocity;
    public Vector3 angularVelocity_body;
    public Vector3 acceleration;
    public Vector3 acceleration_body;
    public Vector3 angularAcceleration;
    public Vector3 angularAcceleration_body;
}

public abstract class PhysicsBody : MonoBehaviour
{
    protected Rigidbody _rigidbody;
    protected BodyState _state;

    public float _mass = 100;
    public Vector3 _cogOffset = new Vector3(0, 0, 0);
    public Vector3 _diagInertia = new Vector3(50, 50, 50);

    // Start is called before the first frame update
    void Start()
    {
        _rigidbody = GetComponent<Rigidbody>();
        _rigidbody.useGravity = false;
        _rigidbody.detectCollisions = false;
        _rigidbody.inertiaTensor = _diagInertia;
        _rigidbody.mass = _mass;
        _rigidbody.centerOfMass = _cogOffset;

        Start_impl();
    }

    // Update is called once per frame
    void Update()
    {

    }

    void FixedUpdate()
    {
        UpdateState();
        ComputePhysics();
    }

    void UpdateState()
    {
        _state = UnityToUnity(_rigidbody, _state, Time.fixedDeltaTime);
    }

    protected abstract void Start_impl();
    protected abstract void ComputePhysics();

    public BodyState UnityToUnity(Rigidbody rigidbody, BodyState previous, float dt)
    {
        BodyState state = new BodyState();

        state.position = rigidbody.position;
        state.rotation = rigidbody.rotation;
        state.velocity = rigidbody.velocity;
        state.velocity_body = transform.InverseTransformDirection(state.velocity);
        state.angularVelocity = rigidbody.angularVelocity;
        state.angularVelocity_body = transform.InverseTransformDirection(state.angularVelocity_body);

        if (previous == null)
        {
            state.acceleration = Vector3.zero;
            state.acceleration_body = Vector3.zero;
            state.angularAcceleration = Vector3.zero;
            state.angularAcceleration_body = Vector3.zero;
        }
        else
        {
            _state.acceleration = (state.velocity - previous.velocity) / dt;
            _state.acceleration_body = (state.velocity_body - previous.velocity_body) / dt;
            _state.angularAcceleration = (state.angularVelocity - previous.angularVelocity) / dt;
            _state.angularAcceleration_body = (state.angularVelocity_body - previous.angularVelocity_body) / dt;
        }

        return state;
    }

    public BodyState UnityToNED(Rigidbody rigidbody, BodyState previous, float dt)
    {
        BodyState state = new BodyState();

        state.position = MathTools.PositionUnityToNED(rigidbody.position);
        state.rotation = MathTools.QuaternionUnityToNED(rigidbody.rotation);
        state.velocity = MathTools.VectorUnityToNED(rigidbody.velocity);
        state.velocity_body = MathTools.VectorUnityToNED(transform.InverseTransformDirection(rigidbody.velocity));
        state.angularVelocity = MathTools.AngularVectorUnityToNED(rigidbody.angularVelocity);
        state.angularVelocity_body = MathTools.AngularVectorUnityToNED(transform.InverseTransformDirection(rigidbody.angularVelocity));

        if (previous == null)
        {
            state.acceleration = Vector3.zero;
            state.acceleration_body = Vector3.zero;
            state.angularAcceleration = Vector3.zero;
            state.angularAcceleration_body = Vector3.zero;
        }
        else
        {
            state.acceleration = (state.velocity - previous.velocity) / dt;
            state.acceleration_body = (state.velocity_body - previous.velocity_body) / dt;
            state.angularAcceleration = (state.angularVelocity - previous.angularVelocity) / dt;
            state.angularAcceleration_body = (state.angularVelocity_body - previous.angularVelocity_body) / dt;
        }

        return state;
    }
}
