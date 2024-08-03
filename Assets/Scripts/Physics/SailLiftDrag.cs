using System;
using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using UnityEngine;
using UnityEngine.UIElements;


namespace Sim.Physics
{
    public class SailLiftDrag : MonoBehaviour, IForceListener, IPhysicsListener
    {
        public float _rho;
        public float _forceScale;
        public float _torqueScale;
        //public float _area;
        //public Vector3 _centerOfEffort;

        public List<float> _areaList;
        public List<AnimationCurve> _liftCurveList;
        public List<AnimationCurve> _dragCurveList;
        //public List<List<float>> _liftCoeffList;
        //public List<List<float>> _dragAngleList;
        //public List<List<float>> _dragCoeffList;
        public List<Vector3> _CoEList;
        public List<float> _minTorque;
        public List<float> _maxTorque;

        public bool _computeOptimalAngle;

        public GameObject _mast;
        public YachtMastController _mastController;

        private IPhysicsManager _physicsManager;
        private IBody _body;
        protected BodyState _state;
        protected WindManager _windManager;
        private RefFrame _refFrame = RefFrame.BODY_NED;//

        private List<LiftDrag> _liftDrag;

        public bool _isDebug;

        // Use this for initialization
        void Start()
        {
            _windManager = WindManager.Instance;

            MeshBasedWaterPhysics boatForcesUnity = GetComponent<MeshBasedWaterPhysics>();

            GameObject[] physicsManager = GameObject.FindGameObjectsWithTag("PhysicsManager");
            _physicsManager = physicsManager[0].GetComponent<IPhysicsManager>();
            _physicsManager.AddPhysicsEventListener(this);
            if (boatForcesUnity.Body != null)
            {
                _body = boatForcesUnity.Body;
                _physicsManager.AddForceListener(this, _body, _refFrame);
            }

            if (_areaList.Count != _liftCurveList.Count ||
               _areaList.Count != _dragCurveList.Count ||
               _areaList.Count != _CoEList.Count)
            {
                Debug.LogError("SailLiftDrag: area / lift / curve, one of them has not the same size");
            }

            _isDebug = true;

            _liftDrag = new List<LiftDrag>();

            for(int i = 0; i < _liftCurveList.Count; i++)
            {
                _liftDrag.Add(new LiftDrag());
                _liftDrag[i] = new LiftDrag();
                _liftDrag[i].LiftCurve = _liftCurveList[i];
                _liftDrag[i].DragCurve = _dragCurveList[i];
                _liftDrag[i].Area = _areaList[i];
                _liftDrag[i].ForceScale = _forceScale;
                _liftDrag[i].TorqueScale = _torqueScale;
                _liftDrag[i].AppliPoint = _CoEList[i];
                _liftDrag[i].IsDebug = _isDebug;
                _liftDrag[i].MinTorque = _minTorque[i];
                _liftDrag[i].MaxTorque = _maxTorque[i];
            }

        }

        private void OnDestroy()
        {
            _body = null;
        }

        private void OnDisable()
        {
            if (_body != null)
            {
                _physicsManager.RemoveForceListener(_body, this);
            }
        }

        private void OnEnable()
        {
            if (_physicsManager == null)
            {
                GameObject[] physicsManager = GameObject.FindGameObjectsWithTag("PhysicsManager");
                _physicsManager = physicsManager[0].GetComponent<IPhysicsManager>();
            }
            if(_body != null )
                _physicsManager.AddForceListener(this, _body, _refFrame);
        }



        // Update is called once per frame
        void Update()
        {
            if (_computeOptimalAngle)
            {
                float angle = FindOptimalAngle(0, 1026);
                _mastController.SetSetpoint(angle);
            }
        }

        protected ForceTorque ComputeSailForce(Vector3 direction, Vector3 velocity, float cx, float area, Vector3 appliPoint)
        {
            ForceTorque force;
            float vMag = velocity.magnitude;

            force.force = 0.5f * _rho * area * cx * Mathf.Abs(vMag) * vMag * direction;
            force.torque = Vector3.Cross(appliPoint, force.force);

            return force;
        }

        Vector3 ComputeApparentWind()
        {
            Vector3 apparentWind = Quaternion.Inverse(_state.rotation) * _windManager.WindVector - _state.velocity_body;

            return apparentWind;
        }

        public void ComputeForce(IBody body, ref ForceTorque force, BodyState state)
        {
            float rho = 1026;

            //for(int i = 0; i < _areaList.Count; i++)
            for(int i = 0; i < 1; i++)
            {
                Vector3 mastDirection_body = Quaternion.Inverse(_state.rotation) * MathTools.VectorUnityToNED(_mast.transform.rotation * Vector3.forward);
                Vector3 fluidVector_body = Quaternion.Inverse(_state.rotation) * MathTools.VectorUnityToNED(_windManager.WindVector);
                // Wind force is less efficient when the boat has heeling (0 for pi/2, 1 for 0 degrees)
                float roll = 0, pitch = 0, yaw = 0;
                MathTools.GetEulerAngleDegrees(_state.rotation, ref roll, ref pitch, ref yaw);
               
                float rollAccount = (90.0f - Mathf.Abs(roll)) / 90.0f;
                // Should take appli point position in account with sail rotation
                force += _liftDrag[i].ComputeForce(_state, fluidVector_body, mastDirection_body, rho) * rollAccount;// _environment.GetRho());
                force.torque.y = 0;

                Debug.Log(mastDirection_body);
                //Debug.Log("ROOOOOOLLLLLLLLL " + roll + " //// " + force.force );


                //// Unity body frame
                //Vector3 apparentWind = ComputeApparentWind();
                //float aoa = ComputeAoA(-apparentWind);
                ////float cl = GetCoeff(aoa, _liftAngleList[i].Evaluate..GetRange(g[i], _liftCoeffList[i]);
                //float cl = GetCoeff(aoa, _liftCurveList[i]);
                //float cd = GetCoeff(aoa, _dragCurveList[i]);
                //Vector3 liftDir = calculateLiftDirection(apparentWind, Vector3.forward);

                ////ForceTorque force = new ForceTorque();
                //Vector3 dragDir = apparentWind.normalized;
                //force += ComputeSailForce(liftDir, apparentWind, cl, _areaList[i], _CoEList[i]);
                //force += ComputeSailForce(dragDir, apparentWind, cd, _areaList[i], _CoEList[i]);
                ////force.force = new Vector3(0,0,40000);
                ////force.torque = new Vector3(0, 0, 0);   
                ////force.force = _state.rotation * force.force;
                ////force.torque = _state.rotation * force.torque;

                
                //if (_isDebug)
                //{
                //    DrawArrow.ForDebug(_state.rotation * _CoEList[i] + _state.position, _state.rotation * liftDir.normalized, Color.red);
                //    DrawArrow.ForDebug(_state.rotation * _CoEList[i] + _state.position, _state.rotation * dragDir.normalized, Color.green);
                //    DrawArrow.ForDebug(_state.rotation * _CoEList[i] + _state.position, force.force.normalized, Color.blue);// _CoEList[i], force.force);
                //}

                //force.force = new Vector3(0, -10000, 0);
                //force.torque += Vector3.Cross(new Vector3(0, 0, -6.88f), force.force);

                //Debug.LogWarning("SO " + force.force + " " + force.torque);
            }
        }

        float FindOptimalAngle(int sailIndex, float rho)
        {
            int optimalAngle = 0;
            float bestForce = 0;
            int maxAngle = 90;
            Vector3 fluidVector_body = Quaternion.Inverse(_state.rotation) * MathTools.VectorUnityToNED(_windManager.WindVector);

            for(int angle = -maxAngle; angle <= maxAngle; angle+=5)
            {
                var quat = Quaternion.Euler(0, angle, 0);
                ForceTorque force = new ForceTorque();
                Vector3 mastDirection_body = /*Quaternion.Inverse(_state.rotation) **/ MathTools.VectorUnityToNED(quat * Vector3.forward);
                
                // Wind force is less efficient when the boat has heeling 
                float rollAccount = (0.5f * Mathf.PI - Mathf.Abs(_state.angularVelocity.x)) / (0.5f * Mathf.PI);
                // Should take appli point position in account with sail rotation
                _liftDrag[sailIndex].IsDebug = false;
                force += _liftDrag[sailIndex].ComputeForce(_state, fluidVector_body, mastDirection_body, rho) * rollAccount;// _environment.GetRho());
                _liftDrag[sailIndex].IsDebug = _isDebug;

                float currentValue = Vector3.Dot(force.force, new Vector3(1, 0, 0));
                if (currentValue > bestForce)
                {
                    bestForce = currentValue;
                    optimalAngle = angle;
                }
            }

            return optimalAngle;
        }

        public void OnPhysicsEvent(IPhysicsListener.EventType eventType, object data)
        {
            if (eventType == IPhysicsListener.EventType.STATE_UPDATED)
            {
                _state = _physicsManager.GetBodyState(_body, RefFrame.NED);
            }
            else if (eventType == IPhysicsListener.EventType.BODY_CREATED)
            {
                _body = (IBody)data;
                _physicsManager.AddForceListener(this, _body, _refFrame);
            }
        }

        public float GetCoeff(float angle, List<float> angles, List<float> coeff)
        {
            //double[] angles = { 0, 12,   15,  17.5,  19,   20,   25,    30,    35,   40,    45,    50,    55,   60,    65,    70,   75,   80,   85 };
            //double[] coeffs = { 0, 0,   0.2,  0.4,   0.6,  0.8,  0.87,  0.92,  0.9,  0.87,  0.83,  0.81,  0.7,  0.58,  0.52,  0.45, 0.37, 0.25, 0.2 };

            int cpt = 0;
            foreach(var val in angles)
                if (angle > val)
                    break;
                else
                    cpt++;
            double t = ((double)angle - angles[cpt - 1]) / (angles[cpt] - angles[cpt - 1]);


            return MathTools.Interp1((float)angles[cpt - 1], (float)angles[cpt], (float)t);
        }

        public float GetCoeff(float angle, AnimationCurve curve)
        {
            return curve.Evaluate(angle);
        }

        public void AddLiftDragListener(LiftDragForceListener listener)
        {
            listener.AddListener(_liftDrag[0]);
            listener.AddListener(_liftDrag[1]);
        }

        // Cl
        // AoA
        // 12   15  17.5  19   20   25    30    35   40    45    50    55   60    65    70    75   80    85
        // 0  0.2   0.4   0.6  0.8  0.87  0.92  0.9  0.87  0.83  0.81  0.7  0.58  0.52  0.45  0.37 0.25  0.2

        // Cl
        // 0    0.2   0.4   0.6  0.8   0.87   0.92  0.9  0.87  0.83  0.81  0.7   0.58  0.52  0.45  0.37  0.25  0.2
        // Drag
        // 0.15 0.15  0.15  0.15 0.15  0.18   0.21  0.3  0.35  0.37  0.41  0.48  0.53  0.62  0.75  0.85  1.0   1.03
    }
}