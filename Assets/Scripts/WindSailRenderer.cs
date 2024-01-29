using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WindSailRenderer : MonoBehaviour
{
    private Cloth _cloth;

    // Start is called before the first frame update
    void Start()
    {
        _cloth = GetComponent<Cloth>();
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 direction = WindManager.Instance.Direction;

        _cloth.externalAcceleration = direction;
        _cloth.randomAcceleration = direction * 2;
    }
}
