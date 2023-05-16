using System.Collections;
using System.Collections.Generic;
using UnityEngine;

class WaterProviderComponent : MonoBehaviour
{
    public IWaterProvider _waterProvider;

    public IWaterProvider WaterProvider { get => _waterProvider; set => _waterProvider = value; }

    //// Start is called before the first frame update
    //void Start()
    //{
        
    //}

    //// Update is called once per frame
    //void Update()
    //{
        
    //}
}
