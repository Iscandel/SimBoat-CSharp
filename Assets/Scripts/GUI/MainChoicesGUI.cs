using Crest;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MainChoicesGUI : MonoBehaviour
{
    protected bool _showCameraButtons;
    protected bool _showVehicleButtons;
    protected bool _showOptionsButtons;

    protected float _windSpeed = 5;
    protected float _windHeading = 360;

    // Start is called before the first frame update
    void Start()
    {
        _showCameraButtons = false;
        _showVehicleButtons = false;
        _showOptionsButtons = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.H))
            CommonGUI._visible = !CommonGUI._visible;
    }

    void OnGUI()
    {
        if(!CommonGUI._visible)
        {
            float w = 150;
            float h = 20;
            GUI.color = Color.black * 0.7f;
            GUI.DrawTexture(new Rect(0, 0, w, h), Texture2D.whiteTexture);
            GUI.color = Color.white;

            GUI.Label(new Rect(5, 2, w, h), "Press H to show options");
            return;
        }

        GUI.skin.toggle.normal.textColor = Color.white;
        GUI.skin.label.normal.textColor = Color.white;

        float _x = 5f, _y = 0f;
        float _h = 100f;

        GUI.color = Color.black * 0.7f;
        GUI.DrawTexture(new Rect(0, 0, CommonGUI._fullWidth + 2f * _x, _h), Texture2D.whiteTexture);
        GUI.color = Color.white;

        GUI.skin.toggle.normal.textColor = Color.white;
        GUI.skin.toggle.fontStyle = FontStyle.Bold;

        GUI.skin.button.normal.textColor = Color.white;
        GUI.skin.button.fontStyle = FontStyle.Bold;

        float width = 150;
        float height = 20;
        float x = 5;
        float y = 5;
        float vspace = 5;
        _showVehicleButtons = GUI.Toggle(new Rect(x, y, width, height), _showVehicleButtons, "Vehicle");// y += h;
        x += width + 5;
        if (_showVehicleButtons) 
        {
            ShowVehicleList(y, width, height, vspace);
        }

        _showCameraButtons = GUI.Toggle(new Rect(x, y, width, height), _showCameraButtons, "Camera");// y += h;
        if( _showCameraButtons )
        {
            float y2= y + height + vspace;
            if (GUI.Button(new Rect(x, y2, width, height), "Free camera"))
                SelectFreeCamera();

            y2 += height + vspace;
            if (GUI.Button(new Rect(x, y2, width, height), "Follow camera"))
                SelectFollowCamera();
        }

        x += width + 5;
        CommonGUI._commonWidth = x + width + 5;
        _showOptionsButtons = GUI.Toggle(new Rect(x, y, width, height), _showOptionsButtons, "Options");// y += h;
        if(_showOptionsButtons)
        {
            float height2 = 15;
            float y2 = y + height + vspace;
            GUI.Label(new Rect(x, y2, width, height), "Wind speed (0 - 10)");
            y2 += height;
            _windSpeed = GUI.HorizontalSlider(new Rect(x, y2, width, height2), _windSpeed, 0, 10);
            WindManager.Instance.Speed = _windSpeed;

            y2 += height2 + vspace;
            GUI.Label(new Rect(x, y2, width, height), "Wind heading (0 - 360)");
            y2 += height;
            _windHeading = GUI.HorizontalSlider(new Rect(x, y2, width, height2), _windHeading, 0, 360);
            WindManager.Instance.Heading = _windHeading;
        }

        CommonGUI._fullWidth = 480 - 2f * _x; ;
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

        name += "State";

        string path = PathResolver.Resolve(name);
        GameObject prefab = Resources.Load<GameObject>(path + name);
        GameObject instance = Instantiate(prefab, Vector3.zero, Quaternion.identity);

        //EntityManager.Instance.RemoveAllEntities();
        //EntityManager.Instance.AddEntity(instance);
    }
}
