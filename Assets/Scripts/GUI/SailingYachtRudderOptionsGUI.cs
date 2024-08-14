using Sim.Physics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SailingYachtRudderOptionsGUI : MonoBehaviour
{
    protected bool _showOptions = false;
    protected bool _automaticMast = true;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
       
    }

    public void OnGUI()
    {
        if (!CommonGUI._visible)
            return;

        GUI.color = Color.white;
        GUI.skin.toggle.normal.textColor = Color.white;
        GUI.skin.toggle.fontStyle = FontStyle.Bold;

        float x = CommonGUI._commonWidth + 10;
        float y = 5;
        float width = 150;
        float height = 20;
        float vspace = 5;
        _showOptions = GUI.Toggle(new Rect(x, y, width + 10, height), _showOptions, "Yacht options");

        CommonGUI._fullWidth = CommonGUI._commonWidth + width;

        if (_showOptions)
        {
            y += height + vspace;
            _automaticMast = GUI.Toggle(new Rect(x, y, width, height), _automaticMast, "Automatic mast");
            ChangeMastOption(_automaticMast);
        }
    }

    public void ChangeMastOption(bool checked_)
    {
        var go = GameObject.FindGameObjectWithTag("Vehicle");
        var sailLiftDrag = go.GetComponent<SailLiftDrag>();
        if (sailLiftDrag != null) 
        {
            sailLiftDrag.ComputeOptimalAngle = checked_;
        }
    }
}
