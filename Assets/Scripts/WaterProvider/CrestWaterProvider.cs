using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;


internal class CrestWaterProvider : IWaterProvider
{
    private float[] _result;
    private Vector3[] _queryResult;
    private float _minLength;

    int[] _hashArray;
    private int _hash;

    private int _queryNumber = 1;
    private int _currentQuery;


    public CrestWaterProvider(float minLength)
    {
        _minLength = minLength;
        _queryNumber = 1;
        _currentQuery = 0;
    }

    public void SetNumberOfQueries(int nb)
    {
        _hashArray = new int[nb];
        _queryNumber = nb;
        for (int i = 0; i < nb; i++)
            _hashArray[i] = HashCode.Combine<CrestWaterProvider, int>(this, i);
    }

    public void SetSamplingTime(float time) { }

    public bool SampleHeightAt(Vector3[] samplePoints, ref float[] heights) 
    {
        var collProvider = Crest.OceanRenderer.Instance?.CollisionProvider;

        if (_result == null || (_result != null && samplePoints.Length != _result.Length))
        {
            _result = new float[samplePoints.Length];
            heights = new float[samplePoints.Length];
            _queryResult = new Vector3[samplePoints.Length];
            //_hashArray = new Vector3[samplePoints.Length];
            //_hash = _hashArray.GetHashCode();
        }

        if (collProvider == null)
        {
            for (int i = 0; i < samplePoints.Length; i++)
                _result[i] = 0;
            return false;
        }

        //GetHashCode()
        var status = collProvider.Query(_hashArray[_currentQuery], _minLength, samplePoints, heights, null, null);

        _currentQuery = _currentQuery + 1 < _queryNumber ? ++_currentQuery : 0; 

        if (!collProvider.RetrieveSucceeded(status))
        {
            for (int i = 0; i < samplePoints.Length; i++)
                _result[i] = Crest.OceanRenderer.Instance.SeaLevel;
            return false;
        }

        //for (int i = 0; i < samplePoints.Length; i++)
        //    heights[i] = _queryResult[i].y + Crest.OceanRenderer.Instance.SeaLevel;

        return true;
    }

    //public float GetHeightAt(int listIndex)
    //{
    //    return _result[listIndex];
    //}

    public float GetHeightAt(Vector3 samplePoint)
    {
        Crest.SampleHeightHelper sampler = new Crest.SampleHeightHelper();
        sampler.Init(samplePoint);
        float height;
        bool status = sampler.Sample(out height);
        if (!status)
            Debug.LogError("Bad height sampling");

        return height;
    }

}

