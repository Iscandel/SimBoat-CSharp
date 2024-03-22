using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WindManager : MonoBehaviour
{
    public Vector3 _direction;
    public float _speed;

    [Tooltip("Angle / North"), Range(0, 360f)]
    public float _heading;

    public bool _isDebug;

    private static WindManager _instance;

    private GameObject _arrow;

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
    { get {
            //float z = Mathf.Cos(_heading * Mathf.Deg2Rad);
            //float x = Mathf.Sqrt(1 - z * z);
            //return new Vector3(x, 0, z);

            return _arrow.transform.rotation * Vector3.forward;
        }
    }

    public float Heading
    { get { return _heading; } }

    public Vector3 WindVector
    { 
        get { return Direction * Speed; } 
    }

    // Start is called before the first frame update
    void Start()
    {
        const string prefabName = "Arrow";
        string path = PathResolver.Resolve(prefabName);
        GameObject prefab = Resources.Load<GameObject>(path + prefabName);
        _arrow = Instantiate(prefab, Vector3.zero, Quaternion.identity);
        _arrow.transform.SetParent(transform);
        _arrow.transform.localPosition = new Vector3(0, 4, 0);
    }

    // Update is called once per frame
    void Update()
    {
        if (_isDebug)
            _arrow.SetActive(true);
        else _arrow.SetActive(false);

        _arrow.transform.rotation = Quaternion.Euler(0, Heading, 0);
    }
}
