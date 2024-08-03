using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Pause : MonoBehaviour
{
    public bool _fixTimeScale = false;
    private bool lastFrameFixTimeScale = false;
    float _previousTimeScale;

    [Tooltip("Angle / North"), Range(0, 2)]
    public float _timeScale;

    // Start is called before the first frame update
    void Start()
    {
        //_previousTimeScale = Time.timeScale;

        _timeScale = Time.timeScale;
        //Time.fixedDeltaTime = 0.01f;
    }

    // Update is called once per frame
    void Update()
    {      
        //if (_fixTimeScale && lastFrameFixTimeScale != _fixTimeScale)
        //{
        //    _previousTimeScale = Time.timeScale;
        //    Time.timeScale = 0;
        //}
        //else if(!_fixTimeScale && lastFrameFixTimeScale != _fixTimeScale)
        //{
        //    Time.timeScale = _previousTimeScale;
        //}

        //lastFrameFixTimeScale = _fixTimeScale;

        Time.timeScale = _timeScale;
    }
}
