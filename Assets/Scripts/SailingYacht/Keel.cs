
using System;
using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography;
using UnityEngine;
using UnityEngine.UIElements;

namespace Sim.Physics
{
    public class Keel : MonoBehaviour, IForceListener, IPhysicsListener
    {

        //public float _scale;
        public float _forceScale;
        public float _torqueScale;
        //public float _area;
        //public Vector3 _centerOfEffort;

        public float _area;
        public AnimationCurve _liftCurve;
        public AnimationCurve _dragCurve;

        public Vector3 _CLR;

        private IPhysicsManager _physicsManager;
        private IBody _body;
        protected BodyState _state;
        protected WindManager _windManager;
        private RefFrame _refFrame = RefFrame.BODY_NED;//

        private LiftDrag _liftDrag;

        public bool _isDebug = true;

        EntityEnvironment _environment;

        // Use this for initialization
        void Start()
        {
            _windManager = WindManager.Instance;

            var boatForcesUnity = GetComponent<Sim.Physics.MeshBasedWaterPhysics>();

            GameObject[] physicsManager = GameObject.FindGameObjectsWithTag("PhysicsManager");
            _physicsManager = physicsManager[0].GetComponent<IPhysicsManager>();
            _physicsManager.AddPhysicsEventListener(this);
            if (boatForcesUnity.Body != null)
            {
                _body = boatForcesUnity.Body;
                _physicsManager.AddForceListener(this, _body, _refFrame);
            }

            //_isDebug = true;

            _liftDrag = new LiftDrag();
            _liftDrag.LiftCurve = _liftCurve;
            _liftDrag.DragCurve = _dragCurve;
            _liftDrag.Area = _area;
            _liftDrag.ForceScale = _forceScale;
            _liftDrag.TorqueScale = _torqueScale;
            _liftDrag.AppliPoint = _CLR;
            _liftDrag.IsDebug = _isDebug;

            _environment = GetComponent<EntityEnvironment>();
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
            if (_body != null)
                _physicsManager.AddForceListener(this, _body, _refFrame);
        }


        // Update is called once per frame
        void Update()
        {

        }

        public void AddLiftDragListener(LiftDragForceListener listener)
        {
            listener.AddListener(_liftDrag);
        }

        public void ComputeForce(IBody body, ref ForceTorque force, BodyState state)
        {
            float rho = _environment.GetRho();

            Vector3 keelDirection_body = /*Quaternion.Inverse(_state.rotation) **/ new Vector3(1, 0, 0);
            Vector3 fluidVector_body = Quaternion.Inverse(_state.rotation) * _environment.GetCurrentVector_WorldNED();
            // Water force is less efficient when the boat has heeling (0 for pi/2, 1 for 0 degrees)
            float roll = 0, pitch = 0, yaw = 0;
            MathTools.GetEulerAngleDegrees(_state.rotation, ref roll, ref pitch, ref yaw);

            float rollAccount = (90.0f - Mathf.Abs(roll)) / 90.0f;
            // Should take appli point position in account with sail rotation
            force += _liftDrag.ComputeForce(_state, fluidVector_body, keelDirection_body, rho);// * rollAccount;
            force.torque.y = 0;
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

        public float GetCoeff(float angle, AnimationCurve curve)
        {
            return curve.Evaluate(angle);
        }
    }

}