using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Pause : MonoBehaviour
{
    public bool _fixTimeScale;
    float _previousTimeScale;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if (_fixTimeScale)
        {
            _previousTimeScale = Time.timeScale;
            Time.timeScale = 0;
        }
        else
            Time.timeScale = _previousTimeScale;
    }
}
