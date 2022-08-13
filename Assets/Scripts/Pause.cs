using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Pause : MonoBehaviour
{
    public bool _fixTimeScale = false;
    private bool lastFrameFixTimeScale = false;
    float _previousTimeScale;
    
    // Start is called before the first frame update
    void Start()
    {
        _previousTimeScale = Time.timeScale;
    }

    // Update is called once per frame
    void Update()
    {      
        if (_fixTimeScale && lastFrameFixTimeScale != _fixTimeScale)
        {
            _previousTimeScale = Time.timeScale;
            Time.timeScale = 0;
        }
        else if(!_fixTimeScale && lastFrameFixTimeScale != _fixTimeScale)
        {
            Time.timeScale = _previousTimeScale;
        }

        lastFrameFixTimeScale = _fixTimeScale;
    }
}
