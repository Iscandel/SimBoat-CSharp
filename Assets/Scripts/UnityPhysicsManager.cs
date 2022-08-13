//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;

//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;

//public interface IPhysicsListener
//{
//    public enum EventType
//    {
//        START,
//        END
//    };

//    public void OnPhysicsEvent(EventType eventType);
//}

//public enum RefFrame
//{
//    WORLD,
//    BODY_FRAME
//}

//public struct ForceTorque
//{
//    public ForceTorque(Vector3 force, Vector3 torque)
//    {
//        this.force = force;
//        this.torque = torque;
//    }

//    public Vector3 force;
//    public Vector3 torque;

//    public static ForceTorque operator +(ForceTorque a, ForceTorque b) => new ForceTorque(a.force + b.force, a.torque + b.torque);
//}

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

//public struct BodyState
//{
//    public Vector3 pos;
//    public Quaternion rot;

//    public Vector3 speed;
//    public Vector3 speed_body;
//    public Vector3 acc;
//    public Vector3 acc_body;
//    public Vector3 angularSpeed;
//    public Vector3 angularSpeed_body;
//    public Vector3 angularAcc;
//    public Vector3 angularAcc_body;
//}

//public struct BodyParams
//{
//    public double mass;
//    public Vector3 diagInertia;
//    public Vector3 cogOffset;
//}
//public class Body
//{
//    public Body()
//    {
//        forceObjects = new List<ForceObject>();
//    }
//    public BodyParams parameters;
//    public ChBodyAuxRef chBody;
//    public BodyState state;
//    public List<ForceObject> forceObjects;
//}

//public class UnityPhysicsManager : MonoBehaviour
//{
//    private ChSystemSMC _chSystem;

//    protected double _oldTime;

//    protected List<Body> _bodies;

//    protected List<IPhysicsListener> _physicsListeners;

//    private static ChronoPhysicsManager _instance;
//    public static ChronoPhysicsManager Instance
//    {
//        get
//        {
//            if (_instance == null)
//            {
//                _instance = FindObjectOfType<ChronoPhysicsManager>();
//            }

//            if (_instance == null)
//            {
//                _instance = new ChronoPhysicsManager();
//            }

//            return _instance;
//        }
//    }

//    ChForce _force;

//    // Start is called before the first frame update
//    void Start()
//    {
//        _chSystem = new ChSystemSMC();
//        _chSystem.Clear();
//        _chSystem.Set_G_acc(new ChVectorD());
//        _chSystem.SetTimestepperType(ChTimestepper.Type.RUNGEKUTTA45);

//        _bodies = new List<Body>();
//        _physicsListeners = new List<IPhysicsListener>();

//        //Initial pos
//        //Vector3 pos = transform.position;
//        //Quaternion unityQuat = transform.rotation;
//        //Vector3 euler = transform.rotation.eulerAngles;

//        ////not correct
//        //ChVectorD chPos = new ChVectorD(pos.x, pos.y, pos.z);
//        //ChQuaternionD quat = new ChQuaternionD(unityQuat.w, unityQuat.z, -unityQuat.x, unityQuat.y);
//        //_chBody.SetFrame_REF_to_abs(new ChFrameD(chPos, quat));

//        //_chSystem.AddBody(_chBody);
//    }

//    //public Vector3 GetSpeed()
//    //{
//    //    ChFrameMovingD chronoFrame = _chBody.GetFrame_REF_to_abs();
//    //    ChVectorD speed = chronoFrame.GetPos_dt();

//    //    return new Vector3((float)speed.x, (float)speed.y, (float)speed.z);
//    //}
//    //public Vector3 GetSpeedBody()
//    //{
//    //    ChFrameMovingD chronoFrame = _chBody.GetFrame_REF_to_abs();
//    //    ChVectorD speed = chronoFrame.GetPos_dt();

//    //    ChVectorD speed_body = chronoFrame.TransformDirectionParentToLocal(chronoFrame.GetPos_dt());
//    //    return new Vector3((float)speed_body.x, (float)speed_body.y, (float)speed_body.z);
//    //}

//    //public void setFirstSimTime(double simTime)
//    //{
//    //    _oldTime = simTime;
//    //}
//    //public void UpdateForcesTorques(double currentTime)
//    //{
//    //    foreach (Body body in _bodies)
//    //    {
//    //        ForceTorque forces = new ForceTorque();
//    //        foreach(ForceObject forceObject in body.forceObjects)
//    //        {
//    //            IForceListener listener = forceObject.listener;
//    //            ForceTorque force = new ForceTorque();
//    //            listener.ComputeForces(body, force, body.state);
//    //            forces += force;

//    //            if (forceObject.frame == RefFrame.WORLD)
//    //            {
//    //                //world space
//    //                forceObject.chForce.SetVRelpoint(bodyCogOffset); or(similar) :
//    //                forceObject.chForce.SetVpoint(body->chBody->GetFrame_REF_to_abs().GetPos());
//    //                forceObject.chForce.SetDir(ChronoTools.Vector3ToChVectorD(forces.force.normalized));
//    //                forceObject.chTorque.SetVpoint(body->chBody->GetFrame_REF_to_abs().GetPos());
//    //                forceObject.chTorque.SetDir(ChronoTools.Vector3ToChVectorD(forces.torque.normalized));
//    //            }
//    //            else
//    //            {
//    //                forceObject.chForce.SetRelDir(ChronoTools.Vector3ToChVectorD(forces.force.normalized));
//    //                forceObject.chTorque.SetRelDir(ChronoTools.Vector3ToChVectorD(forces.torque.normalized));                 
//    //            }

//    //            forceObject.chForce.SetMforce(force.force.magnitude);
//    //            forceObject.chTorque.SetMforce(force.torque.magnitude);
//    //        }           
//    //    }


//    //    _chSystem.DoStepDynamics(currentTime - _oldTime);

//    //    foreach (Body body in _bodies)
//    //    {
//    //        UpdateKinematics(body);
//    //    }
//    //    _oldTime = currentTime;

//    //}


//    public void UpdateForces(double currentTime)
//    {
//        foreach (Body body in _bodies)
//        {
//            body.chBody.Empty_forces_accumulators();
//            foreach (ForceObject forceObject in body.forceObjects)
//            {
//                IForceListener listener = forceObject.listener;
//                List<Force> force = new List<Force>();
//                listener.ComputeForce(body, ref force, body.state);
//                foreach (Force f in force)
//                {
//                    if (forceObject.frame == RefFrame.WORLD)
//                    {
//                        body.chBody.Accumulate_force(ChronoTools.Vector3ToChVectorD(f.force), ChronoTools.Vector3ToChVectorD(f.appliPoint), false);
//                        //forceObject.chForce.SetDir(new ChVectorD(0, -1, 0));
//                        //forceObject.chForce.SetMforce(5);
//                        //ChVectorD v = body.chBody.GetAppliedForce();
//                        //ChForce f2 = body.chBody.GetForceList().ToArray()[0];
//                        //ChVectorD v2 = f2.GetForce();
//                        //_force.SetRelDir(new ChVectorD(0, -1, 0));
//                        //_force.SetMforce(50000);
//                        //body.chBody.SetMass(5);
//                    }
//                    else
//                    {
//                        body.chBody.Accumulate_force(ChronoTools.Vector3ToChVectorD(f.force), ChronoTools.Vector3ToChVectorD(f.appliPoint), true);

//                    }
//                }
//            }
//        }




//        _chSystem.DoStepDynamics(Time.fixedDeltaTime);// 0.001);// Time.fixedDeltaTime);// currentTime - _oldTime);

//        for (int i = 0; i < _bodies.Count; i++)
//        {
//            Body body = _bodies[i];
//            UpdateKinematics(ref body);
//        }
//        _oldTime = currentTime;

//        ////if (currentTime - _oldTime > 0)
//        //{
//        //    Vector3 vectForce = forces.force;
//        //    ChVectorD chforce = new ChVectorD(vectForce.x, vectForce.y, vectForce.z);
//        //    if (_refFrame == RefFrame.WORLD)
//        //        _chForce.SetDir(chforce.GetNormalized());
//        //    else
//        //        _chForce.SetRelDir(chforce.GetNormalized());
//        //    _chForce.SetMforce(chforce.Length());

//        //    _chForce.SetVrelpoint(new ChVectorD(forces.appliPoint.x, forces.appliPoint.y, forces.appliPoint.y));

//        //    //Debug.Log("Applying torques: " + vSum);

//        //    _chSystem.DoStepDynamics(currentTime);// - _oldTime);
//        //    UpdateBodyState();
//        //    _oldTime = currentTime;
//        //}
//    }

//    //void InjectActuatorForces(List<Vector3> forces, List<Vector3> torques)
//    //{
//    //    //_chBody.RemoveAllForces();

//    //    Vector3 vSum = Vector3.zero;
//    //    foreach (Vector3 v in forces)
//    //        vSum += v;
//    //    //Debug.Log("Applying Forces : " + vSum);
//    //    ChVectorD chforce = new ChVectorD(vSum.x, vSum.y, vSum.z);
//    //    if (_refFrame == RefFrame.WORLD)
//    //        _chForce.SetDir(chforce.GetNormalized());
//    //    else
//    //        _chForce.SetRelDir(chforce.GetNormalized());
//    //    _chForce.SetMforce(chforce.Length());


//    //    Vector3 vSumTorque = Vector3.zero;
//    //    foreach (Vector3 v in torques)
//    //        vSumTorque += v;
//    //    //Debug.Log("Applying torques: " + vSum);

//    //    ChVectorD chtorque = new ChVectorD(vSumTorque.x, vSumTorque.y, vSumTorque.z);
//    //    if (_refFrame == RefFrame.WORLD)
//    //        _chTorque.SetDir(chtorque.GetNormalized());
//    //    else
//    //        _chTorque.SetRelDir(chtorque.GetNormalized());
//    //    _chTorque.SetMforce(chtorque.Length());

//    //    //AddForces(_chBody, vSum, vSumTorque);
//    //}
//    // Update is called once per frame
//    void Update()
//    {

//    }

//    private void FixedUpdate()
//    {
//        TriggerPhysicsUpdateEvent(IPhysicsListener.EventType.START);
//        //UpdateForcesTorques(_oldTime + Time.fixedDeltaTime);
//        UpdateForces(_oldTime + Time.fixedDeltaTime);
//        TriggerPhysicsUpdateEvent(IPhysicsListener.EventType.END);
//    }

//    protected void TriggerPhysicsUpdateEvent(IPhysicsListener.EventType eventType)
//    {
//        foreach (IPhysicsListener physicsListener in _physicsListeners)
//        {
//            physicsListener.OnPhysicsEvent(eventType);
//        }
//    }

//    public void AddPhysicsEventListener(IPhysicsListener listener)
//    {
//        _physicsListeners.Add(listener);
//    }

//    public bool RemovePhysicsEventListener(IPhysicsListener listener)
//    {
//        return _physicsListeners.Remove(listener);
//    }

//    public bool SetCurrentBodyState(Body body, Vector3 pos, Quaternion unityQuat)
//    {
//        // TODO handle body speeds

//        ChVectorD chPos = new ChVectorD(pos.x, pos.y, pos.z);
//        //Quaternion unityQuat = Quaternion.Euler(euler);
//        ChQuaternionD quat = new ChQuaternionD(unityQuat.w, unityQuat.x, unityQuat.y, unityQuat.z);

//        body.chBody.SetFrame_REF_to_abs(new ChFrameD(chPos, quat));

//        this.gameObject.transform.position = pos;
//        this.gameObject.transform.eulerAngles = unityQuat.eulerAngles;

//        return true;
//    }

//    void UpdateKinematics(ref Body body)
//    {
//        // /!\ /!\ BodyState is a struct, so enforce reference
//        ref BodyState state = ref body.state;
//        ChFrameMovingD chronoFrame = body.chBody.GetFrame_REF_to_abs();

//        // get body states
//        state.pos = ChronoTools.ChVectorDToVector3(chronoFrame.GetPos());
//        state.rot = ChronoTools.ChQuatToQuat(chronoFrame.GetRot());
//        state.speed = ChronoTools.ChVectorDToVector3(chronoFrame.GetPos_dt());
//        state.acc = ChronoTools.ChVectorDToVector3(chronoFrame.GetPos_dtdt());
//        state.angularSpeed = ChronoTools.ChVectorDToVector3(chronoFrame.GetWvel_par());
//        state.angularAcc = ChronoTools.ChVectorDToVector3(chronoFrame.GetWacc_par());

//        ChVectorD negSpeed_body_old = ChronoTools.Vector3ToChVectorD(-state.speed);
//        ChVectorD speed_body = chronoFrame.TransformDirectionParentToLocal(chronoFrame.GetPos_dt());
//        ChVectorD acceleration_body = new ChVectorD(
//                                            (speed_body.x + negSpeed_body_old.x) / (double)Time.fixedDeltaTime,
//                                            (speed_body.x + negSpeed_body_old.x) / (double)Time.fixedDeltaTime,
//                                            (speed_body.x + negSpeed_body_old.x) / (double)Time.fixedDeltaTime);// / (double)Time.fixedDeltaTime;

//        state.speed_body = ChronoTools.ChVectorDToVector3(speed_body);
//        state.acc_body = ChronoTools.ChVectorDToVector3(acceleration_body);
//        state.angularSpeed_body = ChronoTools.ChVectorDToVector3(chronoFrame.GetWvel_loc());
//        state.angularAcc_body = ChronoTools.ChVectorDToVector3(chronoFrame.GetWacc_loc());

//        //float roll = 0;
//        //float pitch = 0;
//        //float yaw = 0;
//        //GetEulerAngleDegrees(rotation, ref roll, ref pitch, ref yaw);

//        ////Vector3 v3Pos = new Vector3(float.IsNaN((float)chPosition.y) ? 0.0f : (float)chPosition.y,
//        ////                                    float.IsNaN((float)chPosition.z) ? 0.0f : -(float)chPosition.z,
//        ////                                    float.IsNaN((float)chPosition.x) ? 0.0f : (float)chPosition.x);

//        ////Vector3 v3EulerAngles = new Vector3(float.IsNaN(-pitch) ? 0.0f : -pitch,
//        ////                                    float.IsNaN(yaw) ? 0.0f : yaw,
//        ////                                    float.IsNaN(roll) ? 0.0f : -roll);

//        //Vector3 v3Pos = new Vector3(float.IsNaN((float)chPosition.x) ? 0.0f : (float)chPosition.x,
//        //                            float.IsNaN((float)chPosition.y) ? 0.0f : (float)chPosition.y,
//        //                            float.IsNaN((float)chPosition.z) ? 0.0f : (float)chPosition.z);

//        //Vector3 v3EulerAngles = new Vector3(float.IsNaN(roll) ? 0.0f : roll,
//        //                                    float.IsNaN(pitch) ? 0.0f : pitch,
//        //                                    float.IsNaN(yaw) ? 0.0f : yaw);

//        //// Use GetComponent<Transform>() instead TODO
//        //this.gameObject.transform.position = v3Pos;
//        //this.gameObject.transform.eulerAngles = v3EulerAngles;
//    }

//    public void GetEulerAngleDegrees(ChQuaternionD quat, ref float roll, ref float pitch, ref float yaw)
//    {
//        Quaternion unityQuat = new Quaternion((float)quat.e1, (float)quat.e2, (float)quat.e3, (float)quat.e0);

//        Matrix4x4 mat = Matrix4x4.Rotate(unityQuat);
//        Vector3 euler = RotationToEuler(mat, 2, 1, 0);

//        yaw = euler.x * Mathf.Rad2Deg;
//        pitch = euler.y * Mathf.Rad2Deg;
//        roll = euler.z * Mathf.Rad2Deg;

//        if (pitch > 90)
//        {
//            pitch = 180 - pitch;
//            roll += 180;
//            yaw += 180;
//        }
//        else if (pitch < -90)
//        {
//            pitch = -180 - pitch;
//            roll -= 180;
//            yaw -= 180;
//        }

//        roll = (roll > 180.0f) ? (roll - 360.0f) : ((roll < -180.0f) ? (roll + 360.0f) : roll);
//        yaw = (yaw > 180.0f) ? (yaw - 360.0f) : ((yaw < -180.0f) ? (yaw + 360.0f) : yaw);
//    }

//    public Body CreateBody(BodyParams parameters, Vector3 pos, Quaternion rot)
//    {
//        Body body = new Body();
//        body.parameters = parameters;


//        ChBodyAuxRef chBody = new ChBodyAuxRef();
//        chBody.SetMass(parameters.mass);
//        chBody.SetInertiaXX(ChronoTools.Vector3ToChVectorD(parameters.diagInertia));
//        chBody.SetFrame_COG_to_REF(new ChFrameD(ChronoTools.Vector3ToChVectorD(parameters.cogOffset)));
//        chBody.SetCollide(false);

//        ChVectorD chPos = ChronoTools.Vector3ToChVectorD(pos);
//        ChQuaternionD chRot = ChronoTools.QuatToChQuat(rot);
//        chBody.SetFrame_REF_to_abs(new ChFrameD(chPos, chRot));

//        _chSystem.AddBody(chBody);

//        body.chBody = chBody;
//        _bodies.Add(body);

//        return body;
//    }

//    public void addForceListener(IForceListener listener, Body body, RefFrame frame)
//    {
//        ForceObject obj = new ForceObject();
//        obj.listener = listener;
//        obj.frame = frame;
//        initForce(body, ref obj);
//        //obj.chForce = chForce;
//        body.forceObjects.Add(obj);
//    }

//    public void initForce(Body body, ref ForceObject obj)
//    {
//        ChForce chForce = new ChForce();

//        body.chBody.AddForce(chForce);
//        chForce.SetMode(ChForce.ForceType.FORCE);
//        //
//        _force = chForce;

//        ChForce chTorque = new ChForce();
//        body.chBody.AddForce(chTorque);
//        chTorque.SetMode(ChForce.ForceType.TORQUE);

//        BodyParams parameters = body.parameters;

//        ChVectorD negCogOffset = ChronoTools.Vector3ToChVectorD(-parameters.cogOffset);

//        if (obj.frame == RefFrame.WORLD)
//        {
//            chForce.SetAlign(ChForce.AlignmentFrame.WORLD_DIR);
//            chForce.SetFrame(ChForce.ReferenceFrame.WORLD);
//            chForce.SetVpoint(negCogOffset); //TODO In world coords

//            chTorque.SetAlign(ChForce.AlignmentFrame.WORLD_DIR);
//            chTorque.SetFrame(ChForce.ReferenceFrame.WORLD);
//            chTorque.SetVpoint(negCogOffset);//TODO In world coords
//        }
//        else
//        {
//            chForce.SetAlign(ChForce.AlignmentFrame.BODY_DIR);
//            chForce.SetFrame(ChForce.ReferenceFrame.BODY);
//            chForce.SetVrelpoint(negCogOffset);

//            chTorque.SetAlign(ChForce.AlignmentFrame.BODY_DIR);
//            chTorque.SetFrame(ChForce.ReferenceFrame.BODY);
//            chTorque.SetVrelpoint(negCogOffset);
//        }

//        obj.chForce = chForce;
//        obj.chTorque = chTorque;
//    }

//    public Vector3 RotationToEuler(Matrix4x4 matrix, int axis0, int axis1, int axis2)
//    {
//        Vector3 res = new Vector3();

//        int iodd = ((axis0 + 1) % 3 == axis1) ? 0 : 1;
//        bool odd = iodd == 1 ? true : false;
//        int i = axis0;
//        int j = (axis0 + 1 + iodd) % 3;
//        int k = (axis0 + 2 - iodd) % 3;

//        if (axis0 == axis2)
//        {
//            res[0] = Mathf.Atan2(matrix[j, i], matrix[k, i]);
//            if ((odd && res[0] < 0) || ((!odd) && res[0] > 0))
//            {
//                if (res[0] > 0)
//                {
//                    res[0] -= Mathf.PI;
//                }
//                else
//                {
//                    res[0] += Mathf.PI;
//                }
//                float s2 = new Vector2(matrix[j, i], matrix[k, i]).magnitude;
//                res[1] = -Mathf.Atan2(s2, matrix[i, i]);
//            }
//            else
//            {
//                float s2 = new Vector2(matrix[j, i], matrix[k, i]).magnitude;
//                res[1] = Mathf.Atan2(s2, matrix[i, i]);
//            }

//            // With a=(0,1,0), we have i=0; j=1; k=2, and after computing the first two angles,
//            // we can compute their respective rotation, and apply its inverse to M. Since the result must
//            // be a rotation around x, we have:
//            //
//            //  c2  s1.s2 c1.s2                   1  0   0 
//            //  0   c1    -s1       *    M    =   0  c3  s3
//            //  -s2 s1.c2 c1.c2                   0 -s3  c3
//            //
//            //  Thus:  m11.c1 - m21.s1 = c3  &   m12.c1 - m22.s1 = s3

//            float s1 = Mathf.Sin(res[0]);
//            float c1 = Mathf.Cos(res[0]);
//            res[2] = Mathf.Atan2(c1 * matrix[j, k] - s1 * matrix[k, k], c1 * matrix[j, j] - s1 * matrix[k, j]);
//        }
//        else
//        {
//            res[0] = Mathf.Atan2(matrix[j, k], matrix[k, k]);
//            float c2 = new Vector2(matrix[i, i], matrix[i, j]).magnitude;
//            if ((odd && res[0] < 0) || ((!odd) && res[0] > 0))
//            {
//                if (res[0] > 0)
//                {
//                    res[0] -= Mathf.PI;
//                }
//                else
//                {
//                    res[0] += Mathf.PI;
//                }
//                res[1] = Mathf.Atan2(-matrix[i, k], -c2);
//            }
//            else
//                res[1] = Mathf.Atan2(-matrix[i, k], c2);
//            float s1 = Mathf.Sin(res[0]);
//            float c1 = Mathf.Cos(res[0]);
//            res[2] = Mathf.Atan2(s1 * matrix[k, i] - c1 * matrix[j, i], c1 * matrix[j, j] - s1 * matrix[k, j]);
//        }
//        if (!odd)
//            res = -res;

//        return res;
//    }

//    public BodyState GetBodyState(Body body)
//    {
//        return body.state;
//    }
//}

