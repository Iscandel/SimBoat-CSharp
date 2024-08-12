using Crest;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OptionGUI : MonoBehaviour
{
    protected bool _showCameraButtons;
    protected bool _showVehicleButtons;

    // Start is called before the first frame update
    void Start()
    {
        _showCameraButtons = false;
        _showVehicleButtons = false;
    }

    // Update is called once per frame
    void Update()
    {

    }

    void OnGUI()
    {
        GUI.skin.toggle.normal.textColor = Color.white;
        GUI.skin.label.normal.textColor = Color.white;

        float _x = 5f, _y = 0f;
        float _w = 380 - 2f * _x, _h = 100f;

        GUI.color = Color.black * 0.7f;
        GUI.DrawTexture(new Rect(0, 0, _w + 2f * _x, _h), Texture2D.whiteTexture);
        GUI.color = Color.white;

        float width = 150;
        float height = 20;
        float x = 5;
        float y = 5;
        float vspace = 5;
        _showVehicleButtons = GUI.Toggle(new Rect(x, y, width, height), _showVehicleButtons, "Vehicle");// y += h;
        x += width + 5;

        _showCameraButtons = GUI.Toggle(new Rect(x, y, width, height), _showCameraButtons, "Camera");// y += h;

        if(_showVehicleButtons) 
        {
            ShowVehicleList(y, width, height, vspace);
        }

        if( _showCameraButtons )
        {
            y += height + vspace;
            if (GUI.Button(new Rect(x, y, width, height), "Free camera"))
                SelectFreeCamera();

            y += height + vspace;
            if (GUI.Button(new Rect(x, y, width, height), "Follow camera"))
                SelectFollowCamera();
        }
    }

    void SelectFreeCamera()
    {
        var camGO = GameObject.Find("Main Camera");
        var camera = camGO.GetComponent<FreelookCamera>();
        camera.enabled = true;
        camGO.GetComponent<FollowCamera>().enabled = false;

        var entities = EntityManager.Instance.GetEntities();
        if(entities.Count > 0 )
            camGO.GetComponent<FollowCamera>()._followObject = entities[0];
    }

    void SelectFollowCamera()
    {
        var camGO = GameObject.Find("Main Camera");
        var camera = camGO.GetComponent<FreelookCamera>();
        camera.enabled = false;
        camGO.GetComponent<FollowCamera>().enabled = true;
    }

    void ShowVehicleList(float y_, float width, float height, float vspace)
    {
        float x = 5;
        float y = y_;
       
        var list = PathResolver.GetGameObjectsWithTag("Vehicle");
        foreach (var vehicle in list) 
        {
            y += height + vspace;
            if (GUI.Button(new Rect(x, y, width, height), vehicle.name))
                CreateVehicle(vehicle.name);
        } 
    }

    void CreateVehicle(string name)
    {
        SelectFreeCamera();

        string path = PathResolver.Resolve(name);
        GameObject prefab = Resources.Load<GameObject>(path + name);
        GameObject instance = Instantiate(prefab, Vector3.zero, Quaternion.identity);

        EntityManager.Instance.RemoveAllEntities();
        EntityManager.Instance.AddEntity(instance);
    }
}
