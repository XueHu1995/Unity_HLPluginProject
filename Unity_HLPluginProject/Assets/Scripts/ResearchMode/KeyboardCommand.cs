using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeyboardCommand : MonoBehaviour
{
    ResearchModeController rm;
    void Start()
    {
        rm = GetComponent<ResearchModeController>();
    }

    // Update is called once per frame
    void Update()
    {
#if ENABLE_WINMD_SUPPORT

        if (Input.GetKeyDown(KeyCode.S))
        {
            rm.SaveAHATSensorDataEvent();
        }

#endif
    }

    private void OnApplicationFocus(bool focus)
    {

    }
}
