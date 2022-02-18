using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using LightweightMatrixCSharp;
using UnityEngine.UI;

/*! 
 *  \brief     Updates Unity GameObject Transforms
 *  \details   Transforms are updated based on pose matrices passed into the class/script
 *  \author    Hisham Iqbal
 *  \date      2021
 */
public class CaliManagerSimple : MonoBehaviour
{
    //public Text myText;

    public GameObject caliTool;
    public GameObject caliSpheres;

    //public TextMesh debugText;
    [HideInInspector]
    public Matrix4x4 rs_hl = Matrix4x4.identity;


    Matrix4x4 tool_W = Matrix4x4.identity;
    bool _forceToHide = false;
    private KalmanFilter _kalmanFilter;
    private KalmanFilter _kalmanFiltertool;

    Vector4 localstart_tool;
    Vector4 localend_tool;

    private void Start()
    {
        // intialization of KF...
        Matrix _transitionMat = Matrix.IdentityMatrix(7, 7);  // identity
        Matrix _measurementMat = Matrix.IdentityMatrix(7, 7);  // identity
        Matrix _errorCovPostMat = Matrix.IdentityMatrix(7, 7);
        Matrix _processNoiseCovMat = Matrix.IdentityMatrix(7, 7, 1e-5);
        Matrix _measurementNoiseCovMat = Matrix.IdentityMatrix(7, 7, 1e-3);
        _kalmanFilter = new KalmanFilter(_errorCovPostMat, _measurementNoiseCovMat, _processNoiseCovMat, _measurementMat, _transitionMat);

        Matrix _transitionMat_tool = Matrix.IdentityMatrix(6, 6);  // identity
        Matrix _measurementMat_tool = Matrix.IdentityMatrix(6, 6);  // identity
        Matrix _errorCovPostMat_tool = Matrix.IdentityMatrix(6, 6);
        Matrix _measurementNoiseCovMat_tool = Matrix.IdentityMatrix(6, 6, 1e-2);
        Matrix _processNoiseCovMat_tool = Matrix.IdentityMatrix(6, 6, 1e-2);
        _kalmanFiltertool = new KalmanFilter(_errorCovPostMat_tool, _measurementNoiseCovMat_tool, _processNoiseCovMat_tool, _measurementMat_tool, _transitionMat_tool);
        var _initialState = new Matrix(new double[,] { { 0, 0, 0, 0, 0, 0 } }, false);
        _kalmanFiltertool.SetInitialState(_initialState);

        localstart_tool = new Vector4(0, -0.015f, 0.131f, 1);
        localend_tool = new Vector4(0, -0.015f, 0, 1);
    }

    public void ForceToHide()
    {
        _forceToHide = !_forceToHide;
    }

    public void UpdateTrackedTool_simple(double[] matTransforms, Matrix4x4 cameraToWorld)
    {
        int toolID;
        double[] matrixElements = new double[16];
        int numberOfTools = matTransforms.Length / 18; // 1 tool ID + 1 Visibility (0 - False, 1 - True) + 16 mat elements

        for (int i = 0; i < numberOfTools; i++)
        {
            toolID = (int)(matTransforms[i * 18]);

            for (int k = 0; k < 16; k++)
            {
                matrixElements[k] = matTransforms[i * 18 + 2 + k];
            }
            // convert to matrix 
            Matrix4x4 tool_W_RH = MatrixUtilities.FillMatrixWithDoubles(matrixElements,
                MatrixUtilities.MatrixEntryOrder.RowMajor, MatrixUtilities.MatrixUnits.m);
            tool_W = MatrixUtilities.ReturnSwapHandedMatrix(tool_W_RH, MatrixUtilities.Direction.z);  // unity left handed world

            // for static pose calibration

            if (TCPCommunication.message != "" && TCPCommunication.message.Split(',').Length == 13)
            {
                string[] dataSplit = TCPCommunication.message.Split(',');

                // receive plan pose
                Matrix4x4 tool_rs = Matrix4x4.identity;  // SHOULD BE (+ - - )
                tool_rs.SetRow(0, new Vector4(float.Parse(dataSplit[0]), float.Parse(dataSplit[1]), float.Parse(dataSplit[2]), float.Parse(dataSplit[3])));
                tool_rs.SetRow(1, new Vector4(float.Parse(dataSplit[4]), float.Parse(dataSplit[5]), float.Parse(dataSplit[6]), float.Parse(dataSplit[7])));
                tool_rs.SetRow(2, new Vector4(float.Parse(dataSplit[8]), float.Parse(dataSplit[9]), float.Parse(dataSplit[10]), float.Parse(dataSplit[11])));


                // if tracked and correct: calibrate and update calibration tool pose if both cameras see the target
                if (tool_W_RH != Matrix4x4.identity && tool_W.m23 > 0)
                {
                    // convert back to hololens depth camera
                    Matrix4x4 tool_hl = cameraToWorld.inverse * tool_W_RH;
                    rs_hl = tool_hl * tool_rs.inverse;  
                    rs_hl.SetRow(3, new Vector4(0, 0, 0, 1));

                    Vector3 trans = rs_hl.GetColumn(3);
                    Quaternion angles = rs_hl.rotation;

                    if (!_kalmanFilter.initialised)
                    {
                        var _initialState = new Matrix(new double[,] { { trans.x, trans.y, trans.z, angles.x, angles.y, angles.z, angles.w } }, false);
                        _kalmanFilter.SetInitialState(_initialState);
                    }
                    else
                    {
                        var _measurement = new Matrix(new double[,] { { trans.x, trans.y, trans.z, angles.x, angles.y, angles.z, angles.w } }, false);
                        _kalmanFilter.Predict();
                        _kalmanFilter.Correct(_measurement);
                        Vector4 trans_kf = new Vector4((float)_kalmanFilter.X[0, 0], (float)_kalmanFilter.X[1, 0], (float)_kalmanFilter.X[2, 0], 1);
                        Quaternion angle_kf = new Quaternion((float)_kalmanFilter.X[3, 0], (float)_kalmanFilter.X[4, 0], (float)_kalmanFilter.X[5, 0], (float)_kalmanFilter.X[6, 0]);

                        rs_hl = Matrix4x4.Rotate(angle_kf);
                        rs_hl.SetColumn(3, trans_kf);
                    }
                    

                }

                if (rs_hl != Matrix4x4.identity)
                {
                    // show transformed tool
                    Matrix4x4 rscameraToWorld = cameraToWorld * rs_hl;
                    Matrix4x4 poseW = rscameraToWorld * tool_rs;  // right handed
                    Matrix4x4 poseWUnity = MatrixUtilities.ReturnSwapHandedMatrix(poseW, MatrixUtilities.Direction.z);  // left handed

                    caliSpheres.transform.position = poseWUnity.GetColumn(3);  // entry point
                    caliSpheres.transform.rotation = poseWUnity.rotation;  // entry point

                }
                TCPCommunication.message = "";
            }
            else if (TCPCommunication.message != "" && TCPCommunication.message.Split(',').Length == 8)
            {
                string[] dataSplit = TCPCommunication.message.Split(',');

                // tool rigid configuration relative to marker
                localstart_tool.x = float.Parse(dataSplit[0]);
                localstart_tool.y = float.Parse(dataSplit[1]);
                localstart_tool.z = float.Parse(dataSplit[2]);
                localend_tool.x = float.Parse(dataSplit[3]);
                localend_tool.y = float.Parse(dataSplit[4]);
                localend_tool.z = float.Parse(dataSplit[5]);

                _kalmanFiltertool.SetCovMat(Matrix.IdentityMatrix(6, 6, float.Parse(dataSplit[6])), Matrix.IdentityMatrix(6, 6, float.Parse(dataSplit[7])));
                TCPCommunication.message = "";
            }
            else if (!_forceToHide)  // for purely tool tracking
            {
                if (tool_W_RH != Matrix4x4.identity)
                {
                    caliSpheres.transform.position = tool_W.GetColumn(3);  // entry point
                    caliSpheres.transform.rotation = tool_W.rotation;  // entry point

                    Vector3 toolstart = tool_W * localstart_tool;  // head
                    Vector3 toolend = tool_W * localend_tool;  // end

                    var _measurement = new Matrix(new double[,] { { toolstart.x, toolstart.y, toolstart.z, toolend.x, toolend.y, toolend.z } }, false);
                    _kalmanFiltertool.Predict();
                    _kalmanFiltertool.Correct(_measurement);

                    toolstart = new Vector3((float)_kalmanFiltertool.X[0, 0], (float)_kalmanFiltertool.X[1, 0], (float)_kalmanFiltertool.X[2, 0]);
                    toolend = new Vector3((float)_kalmanFiltertool.X[3, 0], (float)_kalmanFiltertool.X[4, 0], (float)_kalmanFiltertool.X[5, 0]);

                    caliTool.transform.position = toolstart;
                    caliTool.transform.LookAt(toolend);
                }


            }
        }
    }

    public Matrix4x4 GetLatestToolPose()
    {
        //List<Vector3> startend = new List<Vector3>(); // empty now
        //startend.Add(toolstart); // adding an example Vector3
        //startend.Add(toolend); // adding an example Vector3

        return tool_W;
    }
}
