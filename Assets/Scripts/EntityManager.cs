using DDS;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class EntityManager : MonoBehaviour
{
    protected List<GameObject> _entities = new List<GameObject>();   

    private static EntityManager _instance;
    public static EntityManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<EntityManager>();
            }

            if (_instance == null)
            {
                GameObject obj = new GameObject("EntityManager");
                obj.AddComponent<EntityManager>();
                _instance = FindObjectOfType<EntityManager>();
            }

            return _instance;
        }
    }
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void AddEntity(GameObject entity) 
    {
        _entities.Add(entity);
    }

    public bool RemoveEntity(GameObject entity) 
    {
        if (entity == null || !_entities.Contains(entity))
            return false;

        Component[] components = entity.GetComponentsInChildren<Component>();
        foreach (var comp in components)
        {
            if (comp as IDisposable != null)
                (comp as IDisposable).Dispose();
        } 
        
        DestroyImmediate(entity);

        _entities.Remove(entity);

        return true;
    }

    public List<GameObject> GetEntities() 
    {
        return _entities;
    }

    public void RemoveAllEntities() 
    {
        foreach (GameObject entity in _entities) 
        {
            Component[] components = entity.GetComponentsInChildren<Component>();
            foreach (var comp in components)
            {
                if (comp as IDisposable != null)
                    (comp as IDisposable).Dispose();
            }
        }

        foreach (GameObject entity in _entities)
        {
            DestroyImmediate(entity);
        }
        
        _entities.Clear();
    }
}
