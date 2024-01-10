
using UnityEngine;

public class FirstScript : MonoBehaviour
{
    public static event System.Action PreUpdate;

    private void Update()
    {
        if (PreUpdate != null)
            PreUpdate();
    }
}