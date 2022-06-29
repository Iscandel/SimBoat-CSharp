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

    public CrestWaterProvider(float minLength)
    {
        _minLength = minLength;
    }

    public bool UpdateSamplingList(Vector3[] samplePoints)
    {
        var collProvider = Crest.OceanRenderer.Instance?.CollisionProvider;

        if (_result == null || (_result != null && samplePoints.Length != _result.Length))
        {
            _result = new float[samplePoints.Length];
            _queryResult = new Vector3[samplePoints.Length];
        }

        if (collProvider == null)
        {
            for (int i = 0; i < samplePoints.Length; i++)
                _result[i] = 0;
            return false;
        }

        var status = collProvider.Query(GetHashCode(), _minLength, samplePoints, _queryResult, null, null);

        if (!collProvider.RetrieveSucceeded(status))
        {
            for (int i = 0; i < samplePoints.Length; i++)
                _result[i] = Crest.OceanRenderer.Instance.SeaLevel;
            return false;
        }

        for (int i = 0; i < samplePoints.Length; i++)
            _result[i] = _queryResult[i].y + Crest.OceanRenderer.Instance.SeaLevel;

        return true;
    }

    public float GetHeightAt(int listIndex)
    {
        return _result[listIndex];
    }

    public float GetHeightAt(Vector3 samplePoint)
    {
        return 0;
    }
}

