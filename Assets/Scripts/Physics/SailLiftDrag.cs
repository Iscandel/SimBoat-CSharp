using System.Collections;
using UnityEngine;

namespace Assets.Scripts.Physics
{
    public class SailLiftDrag : MonoBehaviour, IForceListener, IPhysicsListener
    {
        public float _rho;
        public float _area;
        
        protected WindManager _windManager;
        
        // Use this for initialization
        void Start()
        {
            _windManager = WindManager.Instance;
        }

        // Update is called once per frame
        void Update()
        {

        }
        protected void ComputePhysics(ref ForceTorque forces)
        {
        }

        protected ForceTorque ComputeSailForce(Vector3 velocity, float cx)
        {
            //ForceTorque force;
            //float vMag = velocity.magnitude;

            //float magnitude = 0.5f * _rho * _area * cx * Mathf.Abs(vMag) * vMag;

            //return force;
            throw new System.NotImplementedException();
        }

        public void ComputeForce(IBody body, ref ForceTorque force, BodyState state)
        {
            //Vector3 apparentWind = Quaternion.Inverse(state.rotation) * _windManager.CurrentVector() - state.velocity_body;

            //force += ComputeSailForce();
            throw new System.NotImplementedException();
        }

        public void OnPhysicsEvent(IPhysicsListener.EventType eventType, object data)
        {
            throw new System.NotImplementedException();
        }
    }
}