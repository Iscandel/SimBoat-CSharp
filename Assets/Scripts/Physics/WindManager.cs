using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WindManager : MonoBehaviour
{
    public Vector3 _direction;
    public float _speed;

    private static WindManager _instance;

    public static WindManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<WindManager>();
            }

            if (_instance == null)
            {
                throw new System.Exception("Wind manager should be present in the scene");
            }

            return _instance;
        }
    }

    public float Speed
    { get { return _speed; } set {  _speed = value; } }

    public Vector3 Direction
    { get { return _direction; } set { _direction = value; } }

    // Start is called before the first frame update
    void Start()
    {
       
    }

    // Update is called once per frame
    void Update()
    {
    }
}
