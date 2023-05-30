using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    Camera _camera;
    float _distance;
    Quaternion _rotation;

    public GameObject _followObject;
    float yaw;

    Vector3 _oldMousePos;
    float _mouseSpeed = 10;
    float _scrollSpeed = 20;

    // Start is called before the first frame update
    void Start()
    {
        _distance = 2;
        _camera = GetComponent<Camera>();

        _oldMousePos = Input.mousePosition;
        //_scrollSpeed = 1;
        if(_followObject.GetComponent<BoxCollider>())
        {
            Vector3 bounds = _followObject.GetComponent<BoxCollider>().bounds.size;
            _distance = 2 * Mathf.Max(Mathf.Max(bounds.x, bounds.y), bounds.z);
        }
           
    }

    // Update is called once per frame
    void Update()
    {
        //Vector3 tmp = new Vector3(0, 0, _distance);
        //Vector3 localPos = transform.rotation * tmp;

        //_camera.transform.localPosition = localPos;

        // Rotations (roll and pitch)
        if (Input.GetMouseButtonDown(1))
        {
            _oldMousePos = Input.mousePosition;
        }

        if (Input.GetMouseButton(1))
        {
            Vector3 delta = (Input.mousePosition - _oldMousePos) * Time.deltaTime * _mouseSpeed;
            _oldMousePos = Input.mousePosition;

            _camera.transform.localEulerAngles += new Vector3(-delta.y, delta.x, 0);
           
        }
        
        _distance -= Input.mouseScrollDelta.y * Time.deltaTime* _scrollSpeed;

        _camera.transform.position = _followObject.transform.position - _camera.transform.forward * _distance;
    }
}
