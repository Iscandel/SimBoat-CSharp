using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using JetBrains.Annotations;
using UnityEngine.LowLevel;





//public struct Force
//{
//    public Vector3 force;
//    public Vector3 appliPoint;
//}


//public interface IForceListener
//{
//    public void ComputeForce(Body body, ref List<Force> force, BodyState state);
//}

//public struct ForceObject
//{
//    public IForceListener listener;
//    public RefFrame frame;
//    public ChForce chForce;
//    public ChForce chTorque;
//}

public struct UnityForceObject
{
    public IForceListener listener;
    public RefFrame frame;

}

public class UnityBody : IBody
{
    public UnityBody()
    {
        forceObjects = new List<UnityForceObject>();
    }

    public BodyParams parameters;
    public Rigidbody rigidbody;
    public BodyState state;
    //public BodyState NEDState;
    public List<UnityForceObject> forceObjects;
}

public class UnityPhysicsManager : IPhysicsManager
{
    protected float _oldTime;

    protected List<UnityBody> _bodies;

    protected List<IPhysicsListener> _physicsListeners;

    private static UnityPhysicsManager _instance;
    public static UnityPhysicsManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<UnityPhysicsManager>();
                _instance.Init();
            }

            if (_instance == null)
            {
                _instance = new UnityPhysicsManager();
            }

            return _instance;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        _bodies = new List<UnityBody>();
        _physicsListeners = new List<IPhysicsListener>();


        FirstScript.PreUpdate += PreUpdate;
    }

    void Init()
    {
        _bodies = new List<UnityBody>();
        _physicsListeners = new List<IPhysicsListener>();

        FirstScript.PreUpdate += PreUpdate;
    }

    public void UpdateForces(double currentTime)
    {
        foreach (UnityBody body in _bodies)
        {
            Rigidbody rigidbody = body.rigidbody;
            
            foreach (UnityForceObject forceObject in body.forceObjects)
            {
                IForceListener listener = forceObject.listener;
                //List<ForceTorque> force = new List<ForceTorque>();
                ForceTorque force = new ForceTorque();
                listener.ComputeForce(body, ref force, body.state);
                //foreach (ForceTorque f in force)
                {
                    if (forceObject.frame == RefFrame.WORLD_UNITY || forceObject.frame == RefFrame.WORLD_NED)
                    {
                        Vector3 unityForce = force.force;
                        Vector3 unityTorque = force.torque;
                        if (forceObject.frame == RefFrame.WORLD_NED)
                        {
                            unityForce = MathTools.VectorNEDToUnity(force.force);
                            unityTorque = MathTools.VectorNEDToUnity(force.torque);
                        }
                        rigidbody.AddForceAtPosition(unityForce, rigidbody.worldCenterOfMass + rigidbody.transform.TransformDirection(body.parameters.originFromCoG));
                        rigidbody.AddTorque(unityTorque);
                    }
                    else
                    {
                        Transform transfo = rigidbody.gameObject.transform;
                        Vector3 unityForce;
                        Vector3 unityTorque;
                        if (forceObject.frame == RefFrame.BODY_NED)
                        {
                            unityForce = transfo.TransformDirection(MathTools.VectorNEDToUnity(force.force));
                            unityTorque = transfo.TransformDirection(MathTools.VectorNEDToUnity(force.torque));
                        }
                        else
                        {
                            unityForce = transfo.TransformDirection(force.force);
                            unityTorque = transfo.TransformDirection(force.torque);
                        }
                        rigidbody.AddForceAtPosition(unityForce, rigidbody.worldCenterOfMass + transform.TransformDirection(body.parameters.originFromCoG));
                        rigidbody.AddTorque(unityTorque);
                    }
                }
            }
        }


        //for (int i = 0; i < _bodies.Count; i++)
        //{
        //    UnityBody body = _bodies[i];
        //    UpdateKinematics(ref body);
        //}
        //_oldTime = currentTime;
    }
    
    // Update is called once per frame
    void Update()
    {

    }

    private void FixedUpdate()
    {
        for (int i = 0; i < _bodies.Count; i++)
        {
            UnityBody body = _bodies[i];
            UpdateKinematics(ref body);
        }
        _oldTime = Time.fixedTime;
        TriggerPhysicsUpdateEvent(IPhysicsListener.EventType.STATE_UPDATED, null);
        TriggerPhysicsUpdateEvent(IPhysicsListener.EventType.START, null);
        //UpdateForcesTorques(_oldTime + Time.fixedDeltaTime);
        UpdateForces(_oldTime + Time.fixedDeltaTime);
        TriggerPhysicsUpdateEvent(IPhysicsListener.EventType.END, null);
    }

    private void LateFixedUpdate(float deltaTime)
    {
        //for (int i = 0; i < _bodies.Count; i++)
        //{
        //    UnityBody body = _bodies[i];
        //    UpdateKinematics(ref body);
        //}
        //_oldTime = Time.fixedTime;

    }

    protected void TriggerPhysicsUpdateEvent(IPhysicsListener.EventType eventType, object data)
    {
        foreach (IPhysicsListener physicsListener in _physicsListeners)
        {
            physicsListener.OnPhysicsEvent(eventType, data);
        }
    }

    public override void AddPhysicsEventListener(IPhysicsListener listener)
    {
        _physicsListeners.Add(listener);
    }

    public override bool RemovePhysicsEventListener(IPhysicsListener listener)
    {
        return _physicsListeners.Remove(listener);
    }

    public bool SetCurrentBodyState(IBody body, Vector3 pos, Quaternion quat)
    {
        // TODO handle body speeds

        UnityBody unityBody = body as UnityBody;

        Vector3 unityPos = MathTools.PositionNEDToUnity(pos.x, pos.y, pos.z);
        Quaternion unityQuat = MathTools.QuaternionNEDtoUnity(quat);

        unityBody.rigidbody.gameObject.transform.position = unityPos;
        unityBody.rigidbody.gameObject.transform.eulerAngles = unityQuat.eulerAngles;

        UpdateKinematics(ref unityBody);

        return true;
    }

    protected void PreUpdate()
    {
        //var playerLoop = PlayerLoop.GetCurrentPlayerLoop();
        //var world = World.DefaultGameObjectInjectionWorld;
        //var lateFixedUpdateGroup = world.GetExistingSystem<LateFixedUpdateSystemGroup>();
        //if (lateFixedUpdateGroup != null)
        //    ScriptBehaviourUpdateOrder.AppendSystemToPlayerLoopList(lateFixedUpdateGroup, ref playerLoop, typeof(FixedUpdate));
        //PlayerLoop.SetPlayerLoop(playerLoop);
        // TODO Set script order
        LateFixedUpdate(Time.fixedDeltaTime);
    }

    protected BodyState UnityToUnity(Rigidbody rigidbody, BodyState previous, float dt)
{
        BodyState state = new BodyState();

        state.position = rigidbody.position;
        state.rotation = rigidbody.rotation;
        state.velocity = rigidbody.velocity;
        state.velocity_body = rigidbody.transform.InverseTransformDirection(state.velocity);
        state.angularVelocity = rigidbody.angularVelocity;
        state.angularVelocity_body = rigidbody.transform.InverseTransformDirection(state.angularVelocity_body);

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

        state.worldCenterOfMass = rigidbody.worldCenterOfMass;

        return state;
    }

    protected BodyState UnityToNED(Rigidbody rigidbody, BodyState previous, float dt)
    {
        BodyState state = new BodyState();

        state.position = MathTools.PositionUnityToNED(rigidbody.position);
        state.rotation = MathTools.QuaternionUnityToNED(rigidbody.rotation);
        state.velocity = MathTools.VectorUnityToNED(rigidbody.velocity);
        state.velocity_body = MathTools.VectorUnityToNED(rigidbody.transform.InverseTransformDirection(rigidbody.velocity));
        state.angularVelocity = MathTools.AngularVectorUnityToNED(rigidbody.angularVelocity);
        state.angularVelocity_body = MathTools.AngularVectorUnityToNED(rigidbody.transform.InverseTransformDirection(rigidbody.angularVelocity));

        if (previous == null)
        {
            state.acceleration = Vector3.zero;
            state.acceleration_body = Vector3.zero;
            state.angularAcceleration = Vector3.zero;
            state.angularAcceleration_body = Vector3.zero;
        }
        else if(dt > 0)
        {
            state.acceleration = (state.velocity - previous.velocity) / dt;
            state.acceleration_body = (state.velocity_body - previous.velocity_body) / dt;
            state.angularAcceleration = (state.angularVelocity - previous.angularVelocity) / dt;
            state.angularAcceleration_body = (state.angularVelocity_body - previous.angularVelocity_body) / dt;
        }

        state.worldCenterOfMass = MathTools.PositionUnityToNED(rigidbody.worldCenterOfMass);

        return state;
    }

    protected BodyState UnityStateToNED(BodyState unityState)
    {
        BodyState state = new BodyState();

        state.position = MathTools.PositionUnityToNED(unityState.position);
        state.rotation = MathTools.QuaternionUnityToNED(unityState.rotation);
        state.velocity = MathTools.VectorUnityToNED(unityState.velocity);
        state.velocity_body = MathTools.VectorUnityToNED(unityState.velocity_body);
        state.angularVelocity = MathTools.AngularVectorUnityToNED(unityState.angularVelocity);
        state.angularVelocity_body = MathTools.AngularVectorUnityToNED(state.angularVelocity_body);
        state.acceleration = MathTools.VectorUnityToNED(unityState.acceleration);
        state.acceleration_body = MathTools.VectorUnityToNED(unityState.acceleration_body);
        state.angularAcceleration = MathTools.AngularVectorUnityToNED(unityState.angularAcceleration);
        state.angularAcceleration_body = MathTools.AngularVectorUnityToNED(unityState.angularAcceleration_body);  

        state.worldCenterOfMass = MathTools.PositionUnityToNED(unityState.worldCenterOfMass);

        return state;
    }

    protected void UpdateKinematics(ref UnityBody body)
    {
        // /!\ /!\ BodyState is a struct, so enforce reference
        //ref BodyState state = ref body.state;
        Rigidbody rigidbody = body.rigidbody;

        body.state = UnityToUnity(rigidbody, body.state, Time.fixedTime - _oldTime);
    }

    public void GetEulerAngleDegrees(ChQuaternionD quat, ref float roll, ref float pitch, ref float yaw)
    {
        Quaternion unityQuat = new Quaternion((float)quat.e1, (float)quat.e2, (float)quat.e3, (float)quat.e0);

        Matrix4x4 mat = Matrix4x4.Rotate(unityQuat);
        Vector3 euler = RotationToEuler(mat, 2, 1, 0);

        yaw = euler.x * Mathf.Rad2Deg;
        pitch = euler.y * Mathf.Rad2Deg;
        roll = euler.z * Mathf.Rad2Deg;

        if (pitch > 90)
        {
            pitch = 180 - pitch;
            roll += 180;
            yaw += 180;
        }
        else if (pitch < -90)
        {
            pitch = -180 - pitch;
            roll -= 180;
            yaw -= 180;
        }

        roll = (roll > 180.0f) ? (roll - 360.0f) : ((roll < -180.0f) ? (roll + 360.0f) : roll);
        yaw = (yaw > 180.0f) ? (yaw - 360.0f) : ((yaw < -180.0f) ? (yaw + 360.0f) : yaw);
    }

    public override IBody CreateBody(BodyParams parameters, GameObject bodyGameobject)
    {
        UnityBody body = new UnityBody();
        body.parameters = parameters;

        Rigidbody rigidbody = bodyGameobject.GetComponent<Rigidbody>();

        if (rigidbody != null)
            body.rigidbody = rigidbody;
        else
            body.rigidbody  = bodyGameobject.AddComponent<Rigidbody>();


        body.rigidbody.useGravity = false;
        //rigidbody.detectCollisions = false;
        body.rigidbody.inertiaTensor = parameters.diagInertia;
        body.rigidbody.mass = (float)parameters.mass;
        body.rigidbody.centerOfMass = parameters.localCoMOffset;

        _bodies.Add(body);

        TriggerPhysicsUpdateEvent(IPhysicsListener.EventType.BODY_CREATED, body);
        
        UpdateKinematics(ref body);

        return body;
    }

    public override void AddForceListener(IForceListener listener, IBody body, RefFrame frame)
    {
        UnityBody unityBody = body as UnityBody;
        UnityForceObject obj = new UnityForceObject();
        obj.listener = listener;
        obj.frame = frame;

        unityBody.forceObjects.Add(obj);
    }

    public override void RemoveForceListener(IBody body, IForceListener listener)
    {
        UnityBody unityBody = body as UnityBody;
        unityBody.forceObjects.Remove(unityBody.forceObjects.Find(g => g.listener == listener));
    }

    public Vector3 RotationToEuler(Matrix4x4 matrix, int axis0, int axis1, int axis2)
    {
        Vector3 res = new Vector3();

        int iodd = ((axis0 + 1) % 3 == axis1) ? 0 : 1;
        bool odd = iodd == 1 ? true : false;
        int i = axis0;
        int j = (axis0 + 1 + iodd) % 3;
        int k = (axis0 + 2 - iodd) % 3;

        if (axis0 == axis2)
        {
            res[0] = Mathf.Atan2(matrix[j, i], matrix[k, i]);
            if ((odd && res[0] < 0) || ((!odd) && res[0] > 0))
            {
                if (res[0] > 0)
                {
                    res[0] -= Mathf.PI;
                }
                else
                {
                    res[0] += Mathf.PI;
                }
                float s2 = new Vector2(matrix[j, i], matrix[k, i]).magnitude;
                res[1] = -Mathf.Atan2(s2, matrix[i, i]);
            }
            else
            {
                float s2 = new Vector2(matrix[j, i], matrix[k, i]).magnitude;
                res[1] = Mathf.Atan2(s2, matrix[i, i]);
            }

            // With a=(0,1,0), we have i=0; j=1; k=2, and after computing the first two angles,
            // we can compute their respective rotation, and apply its inverse to M. Since the result must
            // be a rotation around x, we have:
            //
            //  c2  s1.s2 c1.s2                   1  0   0 
            //  0   c1    -s1       *    M    =   0  c3  s3
            //  -s2 s1.c2 c1.c2                   0 -s3  c3
            //
            //  Thus:  m11.c1 - m21.s1 = c3  &   m12.c1 - m22.s1 = s3

            float s1 = Mathf.Sin(res[0]);
            float c1 = Mathf.Cos(res[0]);
            res[2] = Mathf.Atan2(c1 * matrix[j, k] - s1 * matrix[k, k], c1 * matrix[j, j] - s1 * matrix[k, j]);
        }
        else
        {
            res[0] = Mathf.Atan2(matrix[j, k], matrix[k, k]);
            float c2 = new Vector2(matrix[i, i], matrix[i, j]).magnitude;
            if ((odd && res[0] < 0) || ((!odd) && res[0] > 0))
            {
                if (res[0] > 0)
                {
                    res[0] -= Mathf.PI;
                }
                else
                {
                    res[0] += Mathf.PI;
                }
                res[1] = Mathf.Atan2(-matrix[i, k], -c2);
            }
            else
                res[1] = Mathf.Atan2(-matrix[i, k], c2);
            float s1 = Mathf.Sin(res[0]);
            float c1 = Mathf.Cos(res[0]);
            res[2] = Mathf.Atan2(s1 * matrix[k, i] - c1 * matrix[j, i], c1 * matrix[j, j] - s1 * matrix[k, j]);
        }
        if (!odd)
            res = -res;

        return res;
    }

    public override BodyState GetBodyState(IBody body, RefFrame frame)
    {
        if(frame == RefFrame.BODY_NED || frame == RefFrame.WORLD_NED || frame == RefFrame.NED)
            return UnityStateToNED((body as UnityBody).state);
        else
            return (body as UnityBody).state;
    }
}
