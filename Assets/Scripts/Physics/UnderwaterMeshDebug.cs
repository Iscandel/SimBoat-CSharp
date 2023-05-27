using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class UnderwaterMeshDebug : MonoBehaviour
{
    public string _gameObjectName;
    MeshFilter _meshFilter;

    // Start is called before the first frame update
    void Start()
    {
        if (GetComponent<MeshFilter>() == null)
            transform.AddComponent<MeshFilter>();

        GameObject other = GameObject.Find(_gameObjectName);
        if (other != null)
        {
            string underwaterMeshFilterName = "underwater";
            MeshFilter filter = new List<MeshFilter>(other.GetComponents<MeshFilter>()).Find(g => g.name == underwaterMeshFilterName);
            //MeshFilter[] vroum = other.GetComponentsInChildren<MeshFilter>();
            if (filter == null)
                filter = new List<MeshFilter>(other.GetComponentsInChildren<MeshFilter>()).Find(g => g.name == underwaterMeshFilterName);

            if (filter)
                _meshFilter = filter;
        }


        if (GetComponent<MeshRenderer>() == null)
            transform.AddComponent<MeshRenderer>();  
    }

    private void OnEnable()
    {
       
    }



    // Update is called once per frame
    void Update()
    {
        GetComponent<MeshFilter>().mesh = _meshFilter.mesh;
    }
}
