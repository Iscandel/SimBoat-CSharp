using JetBrains.Annotations;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public enum RefFrame
{
    WORLD_UNITY = 1,
    WORLD_NED = 2,
    BODY_UNITY = 4,
    BODY_NED = 8,
    NED = WORLD_NED | BODY_NED,
    UNITY = WORLD_UNITY | BODY_UNITY
}

public struct ForceTorque
{
    //public ForceTorque()
    //{
    //    force = Vector3.zero;
    //    torque = Vector3.zero;
    //}

    public ForceTorque(Vector3 force, Vector3 torque)
    {
        this.force = force;
        this.torque = torque;
    }

    public Vector3 force;
    public Vector3 torque;

    public static ForceTorque operator +(ForceTorque a, ForceTorque b) => new ForceTorque(a.force + b.force, a.torque + b.torque);
}

public struct Force
{
    public Vector3 force;
    public Vector3 appliPoint;
}

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
    public Vector3 worldCenterOfMass;
}

public struct BodyParams
{
    public double mass;
    public Vector3 diagInertia;
    public Vector3 originFromCoG;
    public Vector3 localCoMOffset;
}

public interface IBody { }
public interface IPhysicsListener
{
    public enum EventType
    {
        START,
        END,
        STATE_UPDATED,
        BODY_CREATED
    };

    public void OnPhysicsEvent(EventType eventType, object data);
}

public interface IForceListener
{
    public void ComputeForce(IBody body, ref ForceTorque force, BodyState NEDState);
}

public abstract class IPhysicsManager : MonoBehaviour
{
    public abstract IBody CreateBody(BodyParams parameters, GameObject bodyGameobject);
    public abstract void AddForceListener(IForceListener listener, IBody body, RefFrame frame);
    public abstract void RemoveForceListener(IBody body, IForceListener listener);
    public abstract void AddPhysicsEventListener(IPhysicsListener listener);
    public abstract bool RemovePhysicsEventListener(IPhysicsListener listener);
    public abstract BodyState GetBodyState(IBody body, RefFrame frame);

    public void LateFixedUpdate() { }

    public IEnumerator OnLateFixedUpdate()
    {
        while(true) 
        {
            yield return new WaitForFixedUpdate();
            LateFixedUpdate();
        }
    }

    public void OnEnable()
    {
        StartCoroutine(OnLateFixedUpdate());
    }

    public void OnDisable()
    {
        StopCoroutine(OnLateFixedUpdate());
    }
    //static IPhysicsManager GetPhysicsManager()
    //{
    //}
}
