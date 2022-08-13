using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WaterChunk : MonoBehaviour
{
    protected MeshFilter _meshFilter;

    public float _waveVelocity = 1f;
    public float _wavelength = 5f;
    public float _waterHeight = 1.0f;

    private  SinWaterProvider _provider;

    // Start is called before the first frame update
    void Start()
    {
        this._meshFilter = GetComponent<MeshFilter>();
        WaterGrid.GenerateMesh(_meshFilter.mesh, 200, 1);

        _provider = new SinWaterProvider();

    }

    private void FixedUpdate()
    {
        _provider.SetSamplingTime(Time.fixedTime);
    }

    // Update is called once per frame
    void Update()
    {
        //float time = Time.realtimeSinceStartup;

        WaterController._waveVelocity = _waveVelocity;
        WaterController._wavelength = _wavelength;
        WaterController._waterHeight = _waterHeight;

        Vector3[] vertices = _meshFilter.mesh.vertices;
        for(int i = 0; i < vertices.Length; i++)
        {
            float[] vertexHeight = new float[1];
            _provider.SampleHeightAt(new Vector3[] { vertices[i] }, ref vertexHeight);
            vertices[i].y = vertexHeight[0];
            //vertices[i].y = provider.GetHeightAt(vertices[i]);// WaterController.GetPosY(vertices[i], time);
        }

        _meshFilter.mesh.vertices = vertices;
        _meshFilter.mesh.RecalculateNormals();
        //mesh.UploadMeshData();
    }
}
