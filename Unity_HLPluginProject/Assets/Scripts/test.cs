using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using LightweightMatrixCSharp;

public class test : MonoBehaviour
{
    //private KalmanFilter _kalmanFilter;
    //private void Start()
    //{
    //    Matrix _transitionMat = Matrix.IdentityMatrix(7, 7);  // identity
    //    Matrix _measurementMat = Matrix.IdentityMatrix(7, 7);  // identity
    //    Matrix _errorCovPostMat = Matrix.IdentityMatrix(7, 7);
    //    Matrix _processNoiseCovMat = Matrix.IdentityMatrix(7, 7, 1e-5);
    //    Matrix _measurementNoiseCovMat = Matrix.IdentityMatrix(7, 7, 1e-3);
    //    _kalmanFilter = new KalmanFilter(_errorCovPostMat, _measurementNoiseCovMat, _processNoiseCovMat, _measurementMat, _transitionMat);
    //    // initialise here or at the first frame
    //    var _initialState = new Matrix(new double[,] { { 0, 0, 0, 0, 0, 0 } }, false);
    //    _kalmanFilter.SetInitialState(_initialState);
    //}

    //private void Update()
    //{
    //    if (!_kalmanFilter.initialised)
    //    {
    //        // trans angles should be calculated
    //        var _initialState = new Matrix(new double[,] { { trans.x, trans.y, trans.z, angles.x, angles.y, angles.z, angles.w } }, false);
    //        _kalmanFilter.SetInitialState(_initialState);
    //    }
    //    else
    //    {
    //        // trans angles should be calculated

    //        var _measurement = new Matrix(new double[,] { { trans.x, trans.y, trans.z, angles.x, angles.y, angles.z, angles.w } }, false);
    //        _kalmanFilter.Predict();
    //        _kalmanFilter.Correct(_measurement);
    //        Vector4 trans_kf = new Vector4((float)_kalmanFilter.X[0, 0], (float)_kalmanFilter.X[1, 0], (float)_kalmanFilter.X[2, 0], 1);
    //        Quaternion angle_kf = new Quaternion((float)_kalmanFilter.X[3, 0], (float)_kalmanFilter.X[4, 0], (float)_kalmanFilter.X[5, 0], (float)_kalmanFilter.X[6, 0]);

    //        rs_hl = Matrix4x4.Rotate(angle_kf);
    //        rs_hl.SetColumn(3, trans_kf);
    //    }
    //}
    public LineRenderer lrPlan;  // entry point
    public GameObject planCursor;
    public GameObject planCursor2;  // entry point
    KalmanFilter _kalmanFilter;
    // Start is called before the first frame update
    void Start()
    {
        float pos_diff = (float)1 / 3 - (float)1 / 4;

        Color indcolor = new Color(1, 0, 0); //(RGB)
        //indcolor.r = (float)pos_diff / ((float)1 / 3 - (float)1 / 10);
        indcolor.g = 1 - (float)pos_diff / ((float)1 / 3 - (float)1 / 10);
        planCursor.GetComponent<Renderer>().material.SetColor("_Color", indcolor);

        //planCursor2.GetComponent<Renderer>().material.SetColor("_Color", new Color(0, 1 - pos_diff / (1 / 3 - 1 / 10), 0));
        //planCursor.GetComponent<MeshRenderer>().material.SetColor("_Color", indcolor);


        //planCursor.GetComponent<MeshRenderer>().material.SetColor("_Color", Color.green);
        //Gradient gradientGreen = new Gradient();
        //gradientGreen.SetKeys(new GradientColorKey[] { new GradientColorKey(Color.green, 0.0f), new GradientColorKey(Color.green, 1.0f) }, new GradientAlphaKey[] { new GradientAlphaKey(1.0f, 0.0f), new GradientAlphaKey(1.0f, 1.0f) });
        //lrPlan.colorGradient = gradientGreen;

        //// intialization of KF...
        //Matrix _transitionMat = Matrix.IdentityMatrix(6, 6);  // identity
        //Debug.Log(_transitionMat);
        //Matrix _measurementMat = Matrix.IdentityMatrix(6, 6);  // identity

        //Matrix _processNoiseCovMat = Matrix.IdentityMatrix(6, 6, 1e-4);
        //Matrix _measurementNoiseCovMat = Matrix.IdentityMatrix(6, 6, 1e-3);
        //Matrix _errorCovPostMat = Matrix.IdentityMatrix(6, 6);


        //var _initialState = new Matrix(new double[,] { { 0, 0, 0, 0, 0, 0 } }, false);

        //_kalmanFilter = new KalmanFilter(_errorCovPostMat, _measurementNoiseCovMat, _processNoiseCovMat, _measurementMat, _transitionMat);
        //_kalmanFilter.SetInitialState(_initialState);

    }

    //// Update is called once per frame
    //void Update()
    //{

    //    Vector3 mousePos = Input.mousePosition;
    //    {
    //        planCursor.transform.position = new Vector3(mousePos.x/2000, mousePos.y / 1000, 0.5f);
    //    }

    //    Vector4 localstart_tool = new Vector4(0.025625f, -0.015f, 0.125f, 1);
    //    Vector4 localend_tool = new Vector4(0.025625f, -0.015f, 0, 1);
    //    Matrix4x4 tool_W = planCursor.transform.localToWorldMatrix;

    //    Vector3 toolstart = tool_W * localstart_tool;  // head
    //    Vector3 toolend = tool_W * localend_tool;  // end
    //    Debug.Log(toolstart.ToString("F5") + toolend.ToString("F5"));


    //    Vector3 trans = planCursor.transform.position;
    //    Vector3 angles = planCursor.transform.rotation.eulerAngles;
    //    var _measurement = new Matrix(new double[,] { { toolstart.x, toolstart.y, toolstart.z, toolend.x, toolend.y, toolend.z } }, false);

    //    _kalmanFilter.Predict();
    //    _kalmanFilter.Correct(_measurement);
    //    toolstart = new Vector3((float)_kalmanFilter.X[0, 0], (float)_kalmanFilter.X[1, 0], (float)_kalmanFilter.X[2, 0]);
    //    toolend = new Vector3((float)_kalmanFilter.X[3, 0], (float)_kalmanFilter.X[4, 0], (float)_kalmanFilter.X[5, 0]);
    //    planCursor2.transform.position = toolstart;
    //    planCursor2.transform.LookAt(toolend);

    //    Debug.Log(toolstart.ToString("F5") + toolend.ToString("F5"));
    //    Debug.Log("----------------------");


    //    //Matrix4x4 poseWUnity_kf = Matrix4x4.Rotate(Quaternion.Euler(angle_kf));
    //    //poseWUnity_kf.SetColumn(3, trans_kf);

    //    //planCursor2.transform.position = poseWUnity_kf.GetColumn(3);  // entry point
    //    //planCursor2.transform.rotation = poseWUnity_kf.rotation;  // entry point

    //}
}
