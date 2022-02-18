using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;

#if ENABLE_WINMD_SUPPORT
using HL2UnityPlugin;
#endif

/*! 
 *  \brief     Controller class to interact with ResearchMode DLL.
 *  \details   Modified script which is adapted from online repo by petergu684's HoloLens2-ResearchMode-Unity on GitHub
 *  \author    Hisham Iqbal
 *  \date      2021
 */
public class ResearchModeController : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    HL2ResearchMode researchMode;
#endif

    [System.Serializable]    
    /**
     * A struct to initialise/describe IR-marker equipped tools 
     */
    public struct TrackableTool
    {
        /// <summary>
        /// Integer ID for this tool, user-assigned
        /// </summary>
        public int toolID;
        /// <summary>
        /// A list of triplets corresponding to marker-locations for this tool, units: metres. Should conform to right-handed convention!
        /// </summary>
        public List<Vector3> toolTriplets;
    }


    public List<TrackableTool> toolsToTrack;

    [HideInInspector]
    public Matrix4x4 cameraToWorld;


    CaliManagerSimple toolManager;

    public GameObject depthPreviewPlane = null;
    private Material depthMediaMaterial = null;
    private Texture2D depthMediaTexture = null;
    private byte[] depthFrameData = null;

    public GameObject shortAbImagePreviewPlane = null;
    private Material shortAbImageMediaMaterial = null;
    private Texture2D shortAbImageMediaTexture = null;
    private byte[] shortAbImageFrameData = null;

    // HI Addition
    public GameObject threshAbImagePreviewPlane = null;
    private Material threshAbImageMediaMaterial = null;
    private Texture2D threshAbImageMediaTexture = null;
    private byte[] threshAbImageFrameData = null;

    public GameObject kpAbImagePreviewPlane = null;
    private Material kpAbImageMediaMaterial = null;
    private Texture2D kpAbImageMediaTexture = null;
    private byte[] kpAbImageFrameData = null;
    


    // Start is called before the first frame update
    void Start()
    {
        toolManager = gameObject.GetComponent<CaliManagerSimple>();

        ImageTexturesSetup();

        string toolString = GenerateToolString();
        Debug.Log(toolString); //debug print, can be removed
        ResearchModeSetup(toolString);


    }

    /// <summary>
    /// Function which initialises DLL functions and instructs DLL which tools to track
    /// </summary>
    /// <param name="toolsetString">Formatted string of marker/tool triplet locations</param>
    private void ResearchModeSetup(string toolsetString)
    {
#if ENABLE_WINMD_SUPPORT
        researchMode = new HL2ResearchMode();
        researchMode.InitializeDepthSensor();

        researchMode.SetToolListByString(toolsetString);
        //researchMode.InitializeSpatialCamerasFront();

        researchMode.SetPointCloudDepthOffset(0);

        // Depth sensor should be initialized in only one mode: get world anchore and start loop
        researchMode.StartDepthSensorLoop();
        Debug.Log("researchMode start");
#endif
    }

    /// <summary>
    /// Function will generate a coded formatted string with tool IDs and triplets
    /// </summary>
    /// <returns>Formatted tool-string to be passed into DLL</returns>
    private string GenerateToolString()
    {
        // function will produce a formatted string as follows:
        // "toolID,x1,y1,z1,..xn,yn,zn;toolID2,...."

        string msg = "";

        foreach (TrackableTool tool in toolsToTrack)
        {
            msg += (tool.toolID).ToString() + ",";

            List<Vector3> triplets = tool.toolTriplets;

            for (int i = 0; i < triplets.Count - 1; i++)
            {
                msg += String.Format("{0:F5},{1:F5},{2:F5},", triplets[i].x, triplets[i].y, triplets[i].z);
            }

            // add semi-colon after final triplet
            msg += String.Format("{0:F5},{1:F5},{2:F5};", triplets[triplets.Count - 1].x, triplets[triplets.Count - 1].y, triplets[triplets.Count - 1].z);

        }

        msg.TrimEnd(';');
        return msg;
    }

    /// <summary>
    /// Attaching Unity GameObjects to their corresponding textures used to visualise images passed out of the DLL
    /// </summary>
    private void ImageTexturesSetup()
    {
        depthMediaMaterial = depthPreviewPlane.GetComponent<MeshRenderer>().material;
        depthMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
        depthMediaMaterial.mainTexture = depthMediaTexture;

        shortAbImageMediaMaterial = shortAbImagePreviewPlane.GetComponent<MeshRenderer>().material;
        shortAbImageMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
        shortAbImageMediaMaterial.mainTexture = shortAbImageMediaTexture;

        // HI addition
        threshAbImageMediaMaterial = threshAbImagePreviewPlane.GetComponent<MeshRenderer>().material;
        threshAbImageMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
        threshAbImageMediaMaterial.mainTexture = threshAbImageMediaTexture;

        kpAbImageMediaMaterial = kpAbImagePreviewPlane.GetComponent<MeshRenderer>().material;
        kpAbImageMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
        kpAbImageMediaMaterial.mainTexture = kpAbImageMediaTexture;
        //
    }

    void LateUpdate()
    {
#if ENABLE_WINMD_SUPPORT
        // update depth map texture
        if (researchMode.DepthMapTextureUpdated())
        {
            byte[] frameTexture = researchMode.GetDepthMapTextureBuffer();
            if (frameTexture.Length > 0)
            {
                if (depthFrameData == null)
                {
                    depthFrameData = frameTexture;
                }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, depthFrameData, 0, depthFrameData.Length);
                }

                depthMediaTexture.LoadRawTextureData(depthFrameData);
                depthMediaTexture.Apply();
            }
        }
        // update short-throw AbImage texture
        if (researchMode.ShortAbImageTextureUpdated())
        {
            byte[] frameTexture = researchMode.GetShortAbImageTextureBuffer();
            if (frameTexture.Length > 0)
            {
                if (shortAbImageFrameData == null)
                {
                    shortAbImageFrameData = frameTexture;
                }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, shortAbImageFrameData, 0, shortAbImageFrameData.Length);
                }

                shortAbImageMediaTexture.LoadRawTextureData(shortAbImageFrameData);
                shortAbImageMediaTexture.Apply();
            }
        }

        // HI Addition
        if (researchMode.ThreshAbImageTextureUpdated()) // thresholded image
        {
            byte[] frameTexture = researchMode.GetShortThreshAbImageTextureBuffer();
            if (frameTexture.Length > 0)
            {
                if (threshAbImageFrameData == null) { threshAbImageFrameData = frameTexture; }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, threshAbImageFrameData, 0, threshAbImageFrameData.Length);
                }

                threshAbImageMediaTexture.LoadRawTextureData(threshAbImageFrameData);
                threshAbImageMediaTexture.Apply();
            }
        }

        if (researchMode.KeypointAbImageTextureUpdated()) // labelled keypoint image
        {

            byte[] frameTexture = researchMode.GetShortKeypointAbImageTextureBuffer();
            if (frameTexture.Length > 0)
            {
                if (kpAbImageFrameData == null) { kpAbImageFrameData = frameTexture; }
                else
                {
                    System.Buffer.BlockCopy(frameTexture, 0, kpAbImageFrameData, 0, kpAbImageFrameData.Length);
                }

                kpAbImageMediaTexture.LoadRawTextureData(kpAbImageFrameData);
                kpAbImageMediaTexture.Apply();
            }
        }

        float[] camWorldTransform = researchMode.GetDepthToWorldMatrix();
        cameraToWorld = MatrixUtilities.FillMatrixWithFloats(camWorldTransform, MatrixUtilities.MatrixEntryOrder.ColumnMajor, MatrixUtilities.MatrixUnits.m);

        double[] toolsTransform = researchMode.GetTrackedToolsPoseMatrices();
        toolManager.UpdateTrackedTool_simple(toolsTransform, cameraToWorld);

#endif
    }

    public void SaveAHATSensorDataEvent()
    {
#if ENABLE_WINMD_SUPPORT
        var depthMap = researchMode.GetDepthMapBuffer();
        var AbImage = researchMode.GetShortAbImageBuffer();
        //var depthToWorld = researchMode.GetDepthToWorldMatrix();
#if WINDOWS_UWP
        //tcpClient.SendUINT16Async(depthMap, AbImage);
        ////tcpClient.SendUINT16AsyncWithMatrix(depthMap, AbImage, depthToWorld);
#endif
#endif
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnApplicationFocus(bool focus)
    {
        if (!focus) StopSensorsEvent();
    }

    public void StopSensorsEvent()
    {
#if ENABLE_WINMD_SUPPORT
        researchMode.StopAllSensorDevice();
#endif
    }
}
