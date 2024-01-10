using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WindRenderer : MonoBehaviour
{
    private const int V = 10;
    private GameObject _particlesGameObject;
    private ParticleSystem _particleSystem;
    private readonly string _prefabName = "WindParticles";

    // Start is called before the first frame update
    void Start()
    {
        string path = PathResolver.Resolve(_prefabName);
        GameObject prefab = Resources.Load<GameObject>(path + _prefabName);
        _particlesGameObject = Instantiate(prefab, Vector3.zero, Quaternion.identity);
        _particleSystem = _particlesGameObject.GetComponent<ParticleSystem>();
    }

    // Update is called once per frame
    void Update()
    {
        // Let show the wind on the current camera
        if(_particleSystem != null)
        {
            const float distanceFromCam = 10;
            ParticleSystem.MainModule main = _particleSystem.main;
            main.startSpeedMultiplier = WindManager.Instance.Speed;

            Vector3 cameraPos = Camera.allCameras[0].transform.position;
            Vector3 startPoint = cameraPos - WindManager.Instance.Direction * distanceFromCam;
            ParticleSystem.ShapeModule shape = _particleSystem.shape;
            shape.position = startPoint;

            Quaternion rot = Quaternion.FromToRotation(Vector3.forward, WindManager.Instance.Direction);
            shape.rotation = rot.eulerAngles;
        }
    }
}
