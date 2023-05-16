using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CrestInitialize : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        //WaterProviderComponent component = GetComponent<WaterProviderComponent>();
        //CrestWaterProvider waterProvider = new CrestWaterProvider(0);
        //waterProvider.SetNumberOfQueries(1);
        //component.WaterProvider = waterProvider;
        CrestWaterProvider waterProvider = GetComponent<CrestWaterProvider>();
        waterProvider.SetNumberOfQueries(1);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
