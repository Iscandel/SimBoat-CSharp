using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FreelookCamera : MonoBehaviour
{
    private Camera _camera;
    private float _speed = 1;
    private Vector3 _oldMousePos;
    public float _mouseSpeed;
    public float _scrollSpeed;

    // Start is called before the first frame update
    void Start()
    {
        _camera = GetComponent<Camera>();
        _oldMousePos = Input.mousePosition;
        _mouseSpeed = 10;
        _scrollSpeed = 20;
    }

    // Update is called once per frame
    void Update()
    {
        _speed = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift) ? 10 : 2;
        Quaternion rot = _camera.transform.rotation;

        // Forward, backward, left, right translations
        if (Input.GetKey(KeyCode.UpArrow))
        {
            _camera.transform.localPosition += rot * Vector3.forward * Time.deltaTime * _speed;
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            _camera.transform.localPosition += rot * Vector3.left * Time.deltaTime * _speed;
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            _camera.transform.localPosition += rot * Vector3.right * Time.deltaTime * _speed;
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            _camera.transform.localPosition -= rot * Vector3.forward * Time.deltaTime * _speed;
        }


        // Up, down translations
        if (Input.GetKey(KeyCode.PageUp))
        {
            _camera.transform.localPosition += rot * Vector3.up * Time.deltaTime * _speed;
        }
        if (Input.GetKey(KeyCode.PageDown))
        {
            _camera.transform.localPosition += rot * Vector3.down * Time.deltaTime * _speed;
        }

        // Mouse up / down translations
        if(Input.GetMouseButtonDown(2))
        {
            _oldMousePos = Input.mousePosition;
        }
        if (Input.GetMouseButton(2))
        {
            Vector3 delta = (Input.mousePosition - _oldMousePos) * Time.deltaTime * _speed;
            _oldMousePos = Input.mousePosition;

            _camera.transform.localPosition += rot * new Vector3(-delta.x, -delta.y, 0);
        }
        else
        {

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
        }

        _camera.transform.localPosition += rot * Vector3.forward * Input.mouseScrollDelta.y * Time.deltaTime * _scrollSpeed;
    }
}
