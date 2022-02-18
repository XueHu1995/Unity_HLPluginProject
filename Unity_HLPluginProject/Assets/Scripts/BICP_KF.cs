using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using LightweightMatrixCSharp;


using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.WSA;
using UnityEngine.XR.WSA.Input;
using System.Threading;
using Microsoft.MixedReality.Toolkit.Experimental.Utilities;
using Microsoft.MixedReality.Toolkit.UI;


// App permissions, modify the appx file for research mode streams
// https://docs.microsoft.com/en-us/windows/uwp/packaging/app-capability-declarations

// Reimplement as list loop structure... 
namespace ArUcoDetectionHoloLensUnity
{
    // Using the hololens for cv .winmd file for runtime support
    // Build HoloLensForCV c++ project (x86) and copy all output files
    // to Assets->Plugins->x86
    // https://docs.unity3d.com/2018.4/Documentation/Manual/IL2CPP-WindowsRuntimeSupport.html
    public class BICP_KF : MonoBehaviour
    {

        [Header("Tool/Plan Object")]

        public Text myText;
        public Text myText_rs_hl;

        public GameObject plan;  // entry point
        public GameObject tool;  // entry point
        GameObject tool_head;
        GameObject tool_end;
        public GameObject femurObj;  // entry point

        //public LineRenderer lrLeg;
        //public LineRenderer lrTool;
        //public LineRenderer lrPlan;

        [Header("Femur/Hip Cursor")]
        public GameObject Cursor;  // cursor, red sphere
        GameObject femurCursor;
        GameObject HipCentre;
        public GameObject fadedCursor;  // cursor
        List<GameObject> generated = new List<GameObject>();  // all selected femur

        public Renderer hipStatus;
        public Renderer kfStatus;


        TextMesh ErrInfo; // for error display
        Vector4 localstart;
        Vector4 localend;
        //Vector4 localstart_tool;
        //Vector4 localend_tool;
        Material head;
        Material end;
        Material body;


        float startNavi = 10f; // value at which color start to change
        float goalNavi = 3f; // the actual target error value
        float rangeNavi = 0f;

        bool hipSet = false;

        // research mode matrix
        ResearchModeController controller;
        CaliManagerSimple tooler;
        // Gesture handler
        GestureRecognizer _gestureRecognizer;

        //Gradient gradientGreen = new Gradient();
        //Gradient gradientRed = new Gradient();
        //Gradient gradientBlue = new Gradient();

        bool set_base = false;
        bool base_exist = false;
        Vector3 trans_base = Vector3.zero;
        Quaternion angles_base = Quaternion.identity;
        //Vector3 angles_tool_base = Vector3.zero;
        Vector3 planstart = Vector3.zero;
        Vector3 planend = Vector3.zero;
        float thres_pos = 0.04f;
        float thres_ang = 10.0f;

        private KalmanFilter _kalmanFilter;
        //private KalmanFilter _kalmanFiltertool;

        // Use this for initialization
        async void Start()
        {
            controller = gameObject.GetComponent<ResearchModeController>();
            tooler = gameObject.GetComponent<CaliManagerSimple>();

            tool_head = tool.transform.Find("head").gameObject;
            tool_end = tool.transform.Find("end").gameObject;

            // Initialize gesture handler
            InitializeHandler();


            // global variable preparation
            ErrInfo = plan.transform.Find("ErrInfo").GetComponent<TextMesh>();

            femurCursor = Instantiate(Cursor);
            femurCursor.GetComponent<Renderer>().material.SetColor("_Color", Color.red);  // this is tracked femur centre
            femurCursor.transform.position = new Vector3(0, 0, 0);
            generated.Add(femurCursor);  // current femur position

            HipCentre = Instantiate(Cursor);
            HipCentre.GetComponent<Renderer>().material.SetColor("_Color", Color.yellow);  // this is globally tracked hip centre, not destroyed
            HipCentre.transform.position = new Vector3(0, 0, 0);

            localstart = new Vector4(0.0136f, 0.027f, 0.006f, 1);
            localend = new Vector4(0.0136f, 0.027f, -0.119f, 1);
            //localstart_tool = new Vector4(0.025625f, -0.015f, 0.125f, 1);
            //localend_tool = new Vector4(0.025625f, -0.015f, 0, 1);
            rangeNavi = 1 / Mathf.Abs(goalNavi) - 1 / Mathf.Abs(startNavi);

            head = plan.transform.Find("head").GetComponent<Renderer>().material;
            end = plan.transform.Find("end").GetComponent<Renderer>().material;
            body = plan.transform.Find("body").GetComponent<Renderer>().material;

            // intialization of KF...
            Matrix _transitionMat = Matrix.IdentityMatrix(7, 7);  // identity
            Matrix _measurementMat = Matrix.IdentityMatrix(7, 7);  // identity
            Matrix _errorCovPostMat = Matrix.IdentityMatrix(7, 7);

            Matrix _processNoiseCovMat = Matrix.IdentityMatrix(7, 7, 1e-4);
            Matrix _measurementNoiseCovMat = Matrix.IdentityMatrix(7, 7, 1e-2);
            _kalmanFilter = new KalmanFilter(_errorCovPostMat, _measurementNoiseCovMat, _processNoiseCovMat, _measurementMat, _transitionMat);

            //Matrix _processNoiseCovMat_tool = Matrix.IdentityMatrix(7, 7, 1e-2);
            //_kalmanFiltertool = new KalmanFilter(_errorCovPostMat, _measurementNoiseCovMat, _processNoiseCovMat_tool, _measurementMat, _transitionMat);
            //var _initialState = new Matrix(new double[,] { { 0, 0, 0, 0, 0, 0, 0 } }, false);
            //_kalmanFiltertool.SetInitialState(_initialState);
        }

        Vector3 CorrectAngles(Vector3 angles, Vector3 angles_base)
        {
            // corrected euler angles
            float anglez;
            if (Math.Abs(angles.z - angles_base.z) > 180)
                anglez = 360 - angles.z;
            else
                anglez = angles.z;

            float anglex;
            if (Math.Abs(angles.x - angles_base.x) > 180)
                anglex = 360 - angles.x;
            else
                anglex = angles.x;

            float angley;
            if (Math.Abs(angles.y - angles_base.y) > 180)
                angley = 360 - angles.y;
            else
                angley = angles.y;

            angles = new Vector3(anglex, angley, anglez);

            return angles;
        }

        // Update is called for plan update
        async void Update()
        {
            if (tooler.rs_hl != Matrix4x4.identity)  // latest calibration result
            {
                myText_rs_hl.text = tooler.rs_hl.ToString("F5");  // should be nearly identity
                Matrix4x4 rscameraToWorld = controller.cameraToWorld * tooler.rs_hl;  // right handed, real-time

                // send hip location in real time
                if (hipSet)
                {
                    // hip centre in world -> rsCamera
                    Vector3 HipCentreRH = HipCentre.transform.position;  // left handed
                    HipCentreRH.z *= -1;  // right handed
                    Vector3 hip_rs = (rscameraToWorld.inverse).MultiplyPoint(HipCentreRH);
                    TCPCommunication.SetMessage(VecToString(hip_rs) + ",h");  // in RH system
                }


                if (TCPCommunication.message != "")
                {

                    string[] dataSplit = TCPCommunication.message.Split(',');

                    // get femur location and display in real-time: femur_rs -> femur_w
                    if (dataSplit.Length == 3 && !hipSet)
                    {
                        Vector3 femur_rs = new Vector3(float.Parse(dataSplit[0]), float.Parse(dataSplit[1]), float.Parse(dataSplit[2]));  // right handed
                        Vector3 femur_W = rscameraToWorld.MultiplyPoint(femur_rs);  // right handed
                        femur_W.z *= -1;  // left handed

                        femurCursor.transform.position = femur_W;
                        TCPCommunication.message = "";
                    }
                    // receive calculated hip location
                    else if (dataSplit.Length == 4)
                    {
                        // faded hip at yellow sphere
                        var _hip = Instantiate(fadedCursor);
                        _hip.transform.position = HipCentre.transform.position;
                        generated.Add(_hip);

                        // yellow hip update its location, already in world!
                        Vector3 hip_W = new Vector3(float.Parse(dataSplit[1]), float.Parse(dataSplit[2]), float.Parse(dataSplit[3]));  // already left handed
                        HipCentre.transform.position = hip_W;
                        myText.text = " hip is: " + hip_W.ToString() + "\n";
                        //hipStatus.color = Color.yellow;
                        hipStatus.material.SetColor("_Color", Color.yellow);
                        TCPCommunication.message = "";
                    }



                    // INITIALISATION: set relative plan location, otherwise is origin and unit: 12 + 2 + 2 + 2
                    else if (dataSplit.Length == 10)
                    {
                        // set guidance position: surgical plan in femur coordinate
                        localstart.x = float.Parse(dataSplit[0]);
                        localstart.y = float.Parse(dataSplit[1]);
                        localstart.z = float.Parse(dataSplit[2]);
                        localend.x = float.Parse(dataSplit[3]);
                        localend.y = float.Parse(dataSplit[4]);
                        localend.z = float.Parse(dataSplit[5]);

                        thres_pos = float.Parse(dataSplit[6]);
                        thres_ang = float.Parse(dataSplit[7]);

                        _kalmanFilter.SetCovMat(Matrix.IdentityMatrix(7, 7, float.Parse(dataSplit[8])), Matrix.IdentityMatrix(7, 7, float.Parse(dataSplit[9])));

                        myText.text = myText.text + "INITIALISED! " + thres_pos.ToString("F5") + "," + thres_ang.ToString("F5") + "\n";

                        TCPCommunication.message = "";
                    }


                    // get tracked pose
                    else if (dataSplit.Length == 12)
                    {
                        // receive plan pose + filtering
                        Matrix4x4 pose_rs = Matrix4x4.identity;
                        pose_rs.SetRow(0, new Vector4(float.Parse(dataSplit[0]), float.Parse(dataSplit[1]), float.Parse(dataSplit[2]), float.Parse(dataSplit[3])));
                        pose_rs.SetRow(1, new Vector4(float.Parse(dataSplit[4]), float.Parse(dataSplit[5]), float.Parse(dataSplit[6]), float.Parse(dataSplit[7])));
                        pose_rs.SetRow(2, new Vector4(float.Parse(dataSplit[8]), float.Parse(dataSplit[9]), float.Parse(dataSplit[10]), float.Parse(dataSplit[11])));
                        TCPCommunication.message = "";

                        Matrix4x4 poseW = rscameraToWorld * pose_rs;  // right handed
                        Matrix4x4 poseWUnity = MatrixUtilities.ReturnSwapHandedMatrix(poseW, MatrixUtilities.Direction.z);  // left handed
                        Vector3 trans = poseWUnity.GetColumn(3);
                        Quaternion angles = poseWUnity.rotation;

                        if (set_base)// pulse like
                        {
                            set_base = false;
                            base_exist = true;
                            trans_base = trans;
                            angles_base = angles;

                            var _initialState = new Matrix(new double[,] { { trans_base.x, trans_base.y, trans_base.z, angles_base.x, angles_base.y, angles_base.z, angles_base.w } }, false);
                            _kalmanFilter.SetInitialState(_initialState);
                            myText.text = myText.text + "base set!\n";
                            kfStatus.material.SetColor("_Color", Color.green);

                        }

                        // has to be set first
                        if (base_exist)
                        {
                            //angles = CorrectAngles(angles, angles_base);
                            if (Vector3.Distance(trans, trans_base) < thres_pos && Quaternion.Angle(angles, angles_base) < thres_ang)
                            {
                                var _measurement = new Matrix(new double[,] { { trans.x, trans.y, trans.z, angles.x, angles.y, angles.z, angles.w } }, false);

                                _kalmanFilter.Predict();
                                _kalmanFilter.Correct(_measurement);
                                Vector4 trans_kf = new Vector4((float)_kalmanFilter.X[0, 0], (float)_kalmanFilter.X[1, 0], (float)_kalmanFilter.X[2, 0], 1);
                                Quaternion angle_kf = new Quaternion((float)_kalmanFilter.X[3, 0], (float)_kalmanFilter.X[4, 0], (float)_kalmanFilter.X[5, 0], (float)_kalmanFilter.X[6, 0]);

                                Matrix4x4 poseWUnity_kf = Matrix4x4.Rotate(angle_kf);
                                poseWUnity_kf.SetColumn(3, trans_kf);
                                //myText.text = trans.ToString("F5") + angles.ToString("F5") + "\n" + trans_kf.ToString("F5") + angle_kf.ToString("F5") + "\n";

                                // femur overlay
                                femurObj.transform.position = poseWUnity_kf.GetColumn(3);  // entry point
                                femurObj.transform.rotation = poseWUnity_kf.rotation;  // entry point

                                // plan: related to the initalised plan
                                planstart = poseWUnity_kf * localstart;
                                planend = poseWUnity_kf * localend;

                                //plan.transform.localScale = new Vector3(1, 1, Vector3.Distance(planstart, planend) / 2);
                                plan.transform.position = planstart;        // place bond here
                                plan.transform.LookAt(planend);

                                //kfStatus.color = Color.green;
                                kfStatus.material.SetColor("_Color", Color.green);

                            }
                            // else, world locked
                            else
                                //kfStatus.color = Color.blue;
                                kfStatus.material.SetColor("_Color", Color.yellow);

                        }
                        else
                        // just display it
                        {
                            // femur overlay
                            femurObj.transform.position = poseWUnity.GetColumn(3);  // entry point
                            femurObj.transform.rotation = poseWUnity.rotation;  // entry point

                            // plan: related to the initalised plan
                            Vector3 planstart = poseWUnity * localstart;
                            Vector3 planend = poseWUnity * localend;

                            //plan.transform.localScale = new Vector3(1, 1, Vector3.Distance(planstart, planend) / 2);
                            plan.transform.position = planstart;        // place bond here
                            plan.transform.LookAt(planend);

                            //plan.transform.position = planstart;  // entry point
                            //lrPlan.SetPosition(0, planstart);  // from plan start
                            //lrPlan.SetPosition(1, planend);  // to plan end, render line as entry path
                        }


                    }
                }

                // regardless whether receive tracked pose, display tool and calculate error
                // but only if base exist!
                if (base_exist)
                {
                    Vector3 toolstart = tool_head.transform.position;
                    Vector3 toolend = tool_end.transform.position;

                    // change head cursor colour
                    Vector3 head_err = (toolstart - planstart) * 1000;
                    float head_err_val = head_err.magnitude;
                    if (head_err_val < goalNavi)
                    {
                        head.SetColor("_Color", Color.green);
                    }
                    else if (head_err_val > startNavi)
                    {
                        head.SetColor("_Color", Color.red);
                    }
                    else
                    {
                        float pos_diff = 1 / goalNavi - 1 / head_err_val;
                        Color indcolor = new Color(1, 0, 0); //(RGB)
                        indcolor.g = 1 - pos_diff / rangeNavi;
                        head.SetColor("_Color", indcolor);

                    }


                    // change end cursor colour
                    Vector3 end_err = (toolend - planend) * 1000;
                    float end_err_val = end_err.magnitude;
                    if (end_err_val < goalNavi)
                    {
                        end.SetColor("_Color", new Color(0, 1, 0, 0.67f));
                    }
                    else if (end_err_val > startNavi)
                    {
                        end.SetColor("_Color", new Color(1, 0, 0, 0.67f));
                    }
                    else
                    {
                        float pos_diff = 1 / goalNavi - 1 / end_err_val;
                        Color indcolor = new Color(1, 0, 0, 0.67f); //(RGB)
                        indcolor.g = 1 - pos_diff / rangeNavi;
                        end.SetColor("_Color", indcolor);

                    }
                    ErrInfo.text = "Tip:" + head_err.ToString("F2") + "\n End: " + end_err.ToString("F2") + " mm";

                    // change head cursor colour
                    if (head_err_val < goalNavi && end_err_val < goalNavi)
                    {
                        body.SetColor("_Color", Color.green);
                    }
                    else
                    {
                        body.SetColor("_Color", Color.red);
                    }

                  
                }
            }

        }



        private void InitializeHandler()
        {
            // New recognizer class
            _gestureRecognizer = new GestureRecognizer();

            // set up to receive both tap and double tap events
            _gestureRecognizer.SetRecognizableGestures(GestureSettings.Tap);
            _gestureRecognizer.TappedEvent += (source, tapCount, ray) =>
            {
                if (tapCount == 1)
                {
                    Debug.Log("Tap");
                    var _selectedfemur = Instantiate(Cursor);  // green
                    _selectedfemur.transform.position = femurCursor.transform.position;
                    generated.Add(_selectedfemur);

                    TCPCommunication.SetMessage(VecToString(femurCursor.transform.position));

                }

            };
            _gestureRecognizer.StartCapturingGestures();

            Debug.Log("Gesture recognizer initialized.");
        }


        public void ToggleToolObject()
        {
            femurObj.SetActive(!femurObj.activeSelf);
        }


        public void Delete()
        {
            var _lastfemur = generated[generated.Count - 1];  // green
            Destroy(_lastfemur);
            generated.Remove(_lastfemur);

            TCPCommunication.SetMessage("Delete");
        }

        public void SetKFBase()
        {
            if (!base_exist)  // if not exist, set
            {
                set_base = true;
                myText.text = myText.text + "set KF baseline\n";
            }
            else  // if exist, remove
            {
                base_exist = false;
                //kfStatus.color = Color.red;
                kfStatus.material.SetColor("_Color", Color.red);

                myText.text = myText.text + "remove KF baseline\n";

            }
        }

        public void ConfirmHip()
        {
            Debug.Log("ConfirmHip");

            // clear all displayed obj
            foreach (var obj in generated)
            {
                Destroy(obj);
            }

            // add world anchore
            hipSet = true;
            HipCentre.AddComponent<WorldAnchor>();  // do anchoring
            myText.text = myText.text + "Final Hip in world: " + HipCentre.transform.position.ToString() + "\n";

            // display ar contents
            femurObj.SetActive(true);
            plan.SetActive(true);
            tool.SetActive(true);
            //lrTool.enabled = true;

            //hipStatus.color = Color.green;
            hipStatus.material.SetColor("_Color", Color.green);

        }

        public void onSliderValueChange(SliderEventData eventData)
        {
            float alpha = eventData.NewValue;
            Material material = femurObj.transform.Find("mesh").GetComponent<Renderer>().material;
            Color c = material.color;
            c.a = alpha;
            material.color = c;
        }

        public void RedoHipTracking()
        {
            Debug.Log("RedoHipTracking");
            // let pc know
            TCPCommunication.SetMessage("redo");

            // clear all displayed obj (femur red, femur green, hip faded)
            foreach (var obj in generated)
            {
                Destroy(obj);
            }

            // clear current hip anchore if any exists
            hipSet = false;
            var existingAnchor = HipCentre.GetComponent<WorldAnchor>();
            if (existingAnchor != null)
            {
                DestroyImmediate(existingAnchor);
            }
            HipCentre.transform.position = new Vector3(0, 0, 0);

            // redo femur cursor initialisation for femur tracking
            femurCursor = Instantiate(Cursor);
            femurCursor.GetComponent<Renderer>().material.SetColor("_Color", Color.red);  // this is tracked femur centre
            femurCursor.transform.position = new Vector3(0, 0, 0);
            generated.Add(femurCursor);  // current femur position

            //hide ar display
            femurObj.SetActive(false);
            plan.SetActive(false);
            tool.SetActive(false);

            //lrPlan.enabled = false;
            //lrTool.enabled = false;

            //hipStatus.color = Color.red;
            //kfStatus.color = Color.red;
            hipStatus.material.SetColor("_Color", Color.red);
            //kfStatus.material.SetColor("_Color", Color.red);

            if (base_exist)
                SetKFBase(); // kalman filter needs to be stopped

        }


        Vector3 ExtractTranslationFromMatrix(ref Matrix4x4 matrix)
        {
            Vector3 translate;
            translate.x = matrix.m03;
            translate.y = matrix.m13;
            translate.z = matrix.m23;
            return translate;
        }

        Quaternion ExtractRotationFromMatrix(ref Matrix4x4 matrix)
        {
            Vector3 forward;
            forward.x = matrix.m02;
            forward.y = matrix.m12;
            forward.z = matrix.m22;

            Vector3 upwards;
            upwards.x = matrix.m01;
            upwards.y = matrix.m11;
            upwards.z = matrix.m21;

            return Quaternion.LookRotation(forward, upwards);
        }

        string VecToString(Vector3 vec)
        {
            string ret = vec.x.ToString() + "," + vec.y.ToString() + "," + vec.z.ToString();
            return ret;
        }

    }
}



