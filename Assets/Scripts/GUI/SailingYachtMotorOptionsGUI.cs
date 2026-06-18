using Sim.Physics;
using System.Collections;
using UnityEngine;

public class SailingYachtMotorOptionsGUI : MonoBehaviour
{

    protected bool _showOptions = false;
    protected bool _FEAHydro = false;

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
            _FEAHydro = GUI.Toggle(new Rect(x, y, width, height), _FEAHydro, "FEA hydro.");
            ChangeHydroOption(_FEAHydro);
        }
    }

    public void ChangeHydroOption(bool checked_)
    {
        var go = GameObject.FindGameObjectWithTag("Vehicle"); // Or query entity manager
        var feaHydro = go.GetComponent<MeshBasedWaterPhysics>();
        if (feaHydro != null)
        {
            feaHydro._computePressure = checked_;
            feaHydro._computeViscous = checked_;
        }

        var genericHydro = go.GetComponent<HydrodynamicsDampingSimple>();
        if (genericHydro != null)
        {
            genericHydro.enabled = !checked_;
        }
    }
}