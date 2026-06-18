using System.Collections;
using UnityEngine;

namespace Assets.Scripts
{
    public class SmallBoatInitializer : MonoBehaviour
    {
        string _name = "SmallBoat";

        // Use this for initialization
        void Start()
        {
            
        }

        private void OnEnable()
        {
            var previousInitializer = GameObject.FindGameObjectsWithTag("VehicleInitializer");
            foreach (var init in previousInitializer)
            {
                if (init != this.gameObject)
                    DestroyImmediate(init);
            }

            string path = PathResolver.Resolve(_name);
            GameObject prefab = Resources.Load<GameObject>(path + _name);
            GameObject instance = Instantiate(prefab, Vector3.zero, Quaternion.identity);

            EntityManager.Instance.RemoveAllEntities();
            EntityManager.Instance.AddEntity(instance);
        }

        private void OnDisable()
        {  
        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}