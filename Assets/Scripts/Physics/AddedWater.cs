using Assets.Scripts.Physics;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using UnityEngine;

namespace Sim.Physics
{
    public class AddedWater : MonoBehaviour, IForceListener, IPhysicsListener
    {
        float _surface = 0;
        MeshBasedWaterPhysics _boatForcesUnity;

        protected float _alphaFilteredAcc; 

        protected Vector3 _filteredAccLin;
        protected Vector3 _filteredAccAng;
        public Vector3 _addedWaterLinear;
        public Vector3 _addedWaterAngular;

        private IPhysicsManager _physicsManager;

        private IBody _body;
        private BodyState _state;

        private RefFrame _refFrame = RefFrame.BODY_NED;//

        // Start is called before the first frame update
        void Start()
        {
            _filteredAccLin = Vector3.zero;
            _filteredAccAng = Vector3.zero;

            _boatForcesUnity = GetComponent<MeshBasedWaterPhysics>();

            GameObject[] physicsManager = GameObject.FindGameObjectsWithTag("PhysicsManager");
            _physicsManager = physicsManager[0].GetComponent<IPhysicsManager>();
            //_physicsManager.AddPhysicsEventListener(this);
            if (_boatForcesUnity.Body != null)
            {
                _body = _boatForcesUnity.Body;
                _physicsManager.AddForceListener(this, _body, _refFrame);
            }

            _alphaFilteredAcc = 0.3f;
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

            _physicsManager.AddPhysicsEventListener(this);
        }
        
        private void OnDisable()
        {
            _physicsManager.RemovePhysicsEventListener(this);
            if (_body != null)
            {
                _physicsManager.RemoveForceListener(_body, this);
            }
        }

        // Update is called once per frame
        void Update()
        {

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

        //=============================================================================
        ///////////////////////////////////////////////////////////////////////////////
        (Vector3, Vector3) ComputeFilteredAcceleration(Vector3 acceleration_RVEHLin, Vector3 acceleration_RVEHAng)
        { 
	        _filteredAccLin = (1.0f - _alphaFilteredAcc) * _filteredAccLin + _alphaFilteredAcc * acceleration_RVEHLin;
	        _filteredAccAng = (1.0f - _alphaFilteredAcc) * _filteredAccAng + _alphaFilteredAcc * acceleration_RVEHAng;

	        return (_filteredAccLin, _filteredAccAng);
        }

    //=============================================================================
    ///////////////////////////////////////////////////////////////////////////////
    ForceTorque CalculateForce_RVEH(Vector3 acceleration_RVEH, Vector3 angular_acceleration_RVEH)
    {
        float submergedArea = _boatForcesUnity.SubmergedArea;
        float refSbmergedArea = _boatForcesUnity.RefSubmergedArea;

            Vector3 globalFilteredAcc_RVEHLin, globalFilteredAcc_RVEHAng;
        (globalFilteredAcc_RVEHLin, globalFilteredAcc_RVEHAng) = ComputeFilteredAcceleration(acceleration_RVEH, angular_acceleration_RVEH);

            ForceTorque res = new ForceTorque();
        res.force  = - Vector3.Scale(_addedWaterLinear, globalFilteredAcc_RVEHLin);
        res.torque = - Vector3.Scale(_addedWaterAngular, globalFilteredAcc_RVEHAng);

        if(refSbmergedArea > 0)
            res = res * (submergedArea / refSbmergedArea);

        return res;
    }


    public void ComputeForce(IBody body, ref ForceTorque force, BodyState state)
        {
            force = CalculateForce_RVEH(_state.acceleration_body, _state.angularAcceleration_body);
        }
    }

}