using System.Collections;
using UnityEngine;

namespace Assets.Scripts
{
    public class SailingYachtRudderInitializer : MonoBehaviour
    {
        GameObject _guiGO;
        string _name = "SailingYachtRudder";

        // Use this for initialization
        void Start()
        {
            
        }

        private void OnEnable()
        {
            _guiGO = GameObject.FindGameObjectWithTag("GUI");
            var previousInitializer = GameObject.FindGameObjectsWithTag("VehicleInitializer");
            foreach (var init in previousInitializer)
            {
                if (init != this.gameObject)
                    DestroyImmediate(init);
            }

            // Put GUI options
            if (_guiGO != null)
            {
                _guiGO.AddComponent<SailingYachtRudderOptionsGUI>();
            }

            // Instantiate the vehicle
            string path = PathResolver.Resolve(_name);
            GameObject prefab = Resources.Load<GameObject>(path + _name);
            GameObject instance = Instantiate(prefab, Vector3.zero, Quaternion.identity);

            EntityManager.Instance.RemoveAllEntities();
            EntityManager.Instance.AddEntity(instance);
        }

        private void OnDisable()
        {  
            if (_guiGO != null)
            {
                var guiObj = _guiGO.GetComponent<SailingYachtRudderOptionsGUI>();
                if(guiObj != null)
                {
                    Destroy(guiObj);
                }
            }
        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}