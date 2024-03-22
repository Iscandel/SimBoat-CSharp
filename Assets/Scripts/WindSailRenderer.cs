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
        float speed = WindManager.Instance.Speed;

        _cloth.externalAcceleration = direction * speed;
        _cloth.randomAcceleration = Vector3.one * Mathf.Max(1.0f, speed / 4.0f);
    }
}
