using System;
using System.Collections;
using UnityEngine;

namespace Sim.Physics
{
    public interface LiftDragForceListener
    {
        public void AddListener(LiftDrag liftDrag);
    }

    /// <summary>
    /// Sums all the lift drag force contributions
    /// </summary>
    public class LiftDragDebugManager : MonoBehaviour, LiftDragForceListener, IPhysicsListener
    {
        private Vector3 _lift;
        private Vector3 _drag;
        private Vector3 _force;
        private IPhysicsManager _physicsManager;
        private IBody _body;
        protected BodyState _state;

        public void AddListener(LiftDrag liftDrag)
        {
            liftDrag.OnSailLiftDragComputation += LiftDragComputation;
        }

        // Use this for initialization
        void Start()
        {
            _state = new BodyState();
            GetComponent<SailLiftDrag>().AddLiftDragListener(this);
            GetComponent<Keel>().AddLiftDragListener(this);

            MeshBasedWaterPhysics boatForcesUnity = GetComponent<MeshBasedWaterPhysics>();

            GameObject[] physicsManager = GameObject.FindGameObjectsWithTag("PhysicsManager");
            _physicsManager = physicsManager[0].GetComponent<IPhysicsManager>();
            _physicsManager.AddPhysicsEventListener(this);
            if (boatForcesUnity.Body != null)
            {
                _body = boatForcesUnity.Body;
            }
        }

        // Update is called once per frame
        void FixedUpdate()
        {

            // using script order, make sure this one is the last called
            DrawDebug(_state, _drag, _lift, _force);
            _drag = Vector3.zero;
            _lift = Vector3.zero;
            _force = Vector3.zero;
        }

        void DrawDebug(BodyState state, Vector3 dragDir, Vector3 liftDir, Vector3 force)
        {
            Vector3 appliPointUnity = MathTools.NEDToUnity(state.worldCenterOfMass);

            DrawArrow.ForDebug(appliPointUnity, MathTools.VectorNEDToUnity(state.rotation * dragDir.normalized), Color.green);
            DrawArrow.ForDebug(appliPointUnity, MathTools.VectorNEDToUnity(state.rotation * liftDir.normalized), Color.red);
            DrawArrow.ForDebug(appliPointUnity, MathTools.VectorNEDToUnity(state.rotation * force.normalized), Color.blue);

            Debug.Log("===============================================");
            Debug.Log("Total");
            Debug.Log("Drag vector: " + dragDir);
            Debug.Log("Lift vector: " + liftDir);
            Debug.Log("total vector: " + force);
        }

        void LiftDragComputation(Vector3 liftDir, Vector3 dragDir, Vector3 force)
        {
            _lift += liftDir;
            _drag += dragDir;
            _force += force;
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
            }
        }
    }

}