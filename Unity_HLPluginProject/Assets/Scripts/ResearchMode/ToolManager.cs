using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/*! 
 *  \brief     Updates Unity GameObject Transforms
 *  \details   Transforms are updated based on pose matrices passed into the class/script
 *  \author    Hisham Iqbal
 *  \date      2021
 */
public class ToolManager : MonoBehaviour
{
    private readonly object trackedToolsLock = new object();

    /// <summary>
    /// Custom struct to describe IR-marker equipped tools
    /// </summary>
    [System.Serializable]
    public struct ToolUnityStruct
    {
        /// <summary>
        /// A descriptor integer ID for this tool
        /// </summary>
        public int toolID;
        
        /// <summary>
        /// Unity transform corresponding to the tool
        /// </summary>
        public Transform ToolObjectTransform;

        /// <summary>
        /// Unity GameObject corresponding to the tool, not necessary
        /// </summary>
        public GameObject ToolObjectGameObject;

        [HideInInspector]
        /// <summary>
        /// Transform matrix describing the pose of the tool w.r.t the holographic world-frame. Left-handed
        /// </summary>
        public Matrix4x4 Tool_HoloFrame_LH;

        [HideInInspector]
        /// <summary>
        /// Bool indicating if the tool is visible to the headset
        /// </summary>
        public bool VisibleToHoloLens;

        [HideInInspector]
        /// <summary>
        /// Timestamp relative to app startup of when the tool was last seen by the headset
        /// </summary>
        public float TimestampLastSeen;

    }


    System.Diagnostics.Stopwatch ScriptTimer = new System.Diagnostics.Stopwatch();

    /// <summary>
    /// Internal dictionary object, keys: integer tool id. values: ToolUnityStruct for the tool
    /// </summary>
    Dictionary<int, ToolUnityStruct> UnityToolsDictionary = new Dictionary<int, ToolUnityStruct>();

    /// <summary>
    /// Class/script specific list, will be shown in Unity inspector, where user can assign the ToolObjectTransform/ToolObjectGameObject fields 
    /// </summary>
    public List<ToolUnityStruct> TrackedTools;



    // Start is called before the first frame update
    void Start()
    {
        PopulateUnityTools();
        ScriptTimer.Start();        
    }

    /// <summary>
    /// Function initialising the internal UnityToolsDictionary based on user selected TrackedTools properties set in Unity inspector
    /// </summary>
    void PopulateUnityTools()
    {
        // grab TrackedTools from Unity inspector and populate
        foreach (ToolUnityStruct item in TrackedTools)
        {
            ToolUnityStruct tool = item;
            tool.Tool_HoloFrame_LH = Matrix4x4.identity;
            tool.TimestampLastSeen = 0f;            
            UnityToolsDictionary.Add(item.toolID, tool);

        }
    }

    // Update is called once per frame
    void Update()
    {
        UpdateHologramPositions();
    }

    /// <summary>
    /// Function which updates UnityToolDictionary elements' Unity transforms based on their visibility and calculated pose matrices
    /// </summary>
    void UpdateHologramPositions()
    {
        float currentTimestamp;

        foreach (var tool in UnityToolsDictionary.ToArray())
        {
            currentTimestamp = ScriptTimer.ElapsedMilliseconds;

            // variable names for easier reading + handling
            bool holoVisible;
            Matrix4x4 tool_Holo;
            Transform tool_transform;
            float tool_lastSeen = 0f;
            ToolUnityStruct toolValues;

            lock (trackedToolsLock)
            {
                toolValues = tool.Value;
            }

            // cached values this frame
            holoVisible = toolValues.VisibleToHoloLens;
            tool_Holo = toolValues.Tool_HoloFrame_LH;

            tool_transform = toolValues.ToolObjectTransform;
            tool_lastSeen = toolValues.TimestampLastSeen;

            // update actuall transformation
            if (holoVisible)
            {
                tool_transform.rotation = tool_Holo.rotation;
                tool_transform.position = tool_Holo.GetColumn(3);
            }
            else
            {               
                if ((currentTimestamp - tool_lastSeen) > 3000f) //3s undetected tool
                {
                    // reset tool pose
                    tool_transform.rotation = Quaternion.identity;
                    tool_transform.position = Vector3.zero;
                }
            }

            lock (trackedToolsLock)
            {
                if (UnityToolsDictionary.ContainsKey(tool.Key)) UnityToolsDictionary[tool.Key] = toolValues;
            }
        }
    }

    /// <summary>
    /// Externally accessible function for updating the internal UnityToolsDictionary based on raw double array
    /// </summary>
    /// <param name="matTransforms">Formatted double packet</param>
    public void UpdateTrackedTool_HoloDictionary(double[] matTransforms)
    {
        int toolID;
        bool visible2Holo = false;
        double[] matrixElements = new double[16];

        // expected packet format:
        // [1 tool ID, 1 Visibility (0 - False, 1 - True), 16 mat elements]
        int numberOfTools = matTransforms.Length / 18; // 1 tool ID + 1 Visibility (0 - False, 1 - True) + 16 mat elements

        for (int i = 0; i < numberOfTools; i++)
        {
            toolID = (int)(matTransforms[i * 18]);
            visible2Holo = ((int)(matTransforms[i * 18 + 1]) != 0);

            for (int k = 0; k < 16; k++)
            {
                matrixElements[k] = matTransforms[i * 18 + 2 + k];
            }

            Matrix4x4 tool_Holo_RH = MatrixUtilities.FillMatrixWithDoubles(matrixElements,
                MatrixUtilities.MatrixEntryOrder.RowMajor, MatrixUtilities.MatrixUnits.m);

            Matrix4x4 tool_Holo_LH = MatrixUtilities.ReturnSwapHandedMatrix(tool_Holo_RH,
                MatrixUtilities.Direction.z);
            
            ToolUnityStruct tool;
            lock (trackedToolsLock)
            {
                tool = UnityToolsDictionary[(int)toolID];
                tool.Tool_HoloFrame_LH = tool_Holo_LH;

                tool.VisibleToHoloLens = visible2Holo;

                if (visible2Holo) tool.TimestampLastSeen = ScriptTimer.ElapsedMilliseconds;
                UnityToolsDictionary[(int)toolID] = tool;
            }
        }
    }
}
