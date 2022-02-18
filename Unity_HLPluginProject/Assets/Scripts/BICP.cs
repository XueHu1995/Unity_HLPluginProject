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


// App permissions, modify the appx file for research mode streams
// https://docs.microsoft.com/en-us/windows/uwp/packaging/app-capability-declarations

// Reimplement as list loop structure... 
namespace ArUcoDetectionHoloLensUnity
{
    // Using the hololens for cv .winmd file for runtime support
    // Build HoloLensForCV c++ project (x86) and copy all output files
    // to Assets->Plugins->x86
    // https://docs.unity3d.com/2018.4/Documentation/Manual/IL2CPP-WindowsRuntimeSupport.html
    public class BICP : MonoBehaviour
    {

        [Header("Tool/Plan Object")]

        public Text myText;
        public Text myText_rs_hl;

        public GameObject plan;  // entry point
        public GameObject tool;  // entry point
        public GameObject femurObj;  // entry point

        [Header("Femur/Hip Cursor")]
        public GameObject Cursor;  // cursor, red sphere
        GameObject femurCursor;
        GameObject HipCentre;
        public GameObject fadedCursor;  // cursor
        List<GameObject> generated = new List<GameObject>();  // all selected femur
        public Image hipStatus;
        public Image kfStatus;

        TextMesh ErrInfo; // for error display
        Vector4 localstart;
        Vector4 localend;
        Vector4 localstart_tool;
        Vector4 localend_tool;

        bool hipSet = false;

        // research mode matrix
        ResearchModeController controller;
        CaliManagerSimple tooler;
        // Gesture handler
        GestureRecognizer _gestureRecognizer;


        bool set_base = false;
        bool base_exist = false;
        Vector3 trans_base = Vector3.zero;
        Vector3 angles_base = Vector3.zero;

        Vector3 planstart = Vector3.zero;
        Vector3 planend = Vector3.zero;
        float thres_pos = 0.03f;
        float thres_ang = 10.0f;

        Vector3 trans_last;
        Quaternion quats_last;
        Vector3 trans_last_tool = Vector3.zero;
        Quaternion quats_last_tool = Quaternion.identity;



        //private KalmanFilter _kalmanFilter;
        //private KalmanFilter _kalmanFiltertool;

        // Use this for initialization
        async void Start()
        {
            controller = gameObject.GetComponent<ResearchModeController>();
            tooler = gameObject.GetComponent<CaliManagerSimple>();

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

            localstart = new Vector4(0, 0, 0, 1);
            localend = new Vector4(0, 0, -1, 1);
            localstart_tool = new Vector4(0, 0, 0, 1);
            localend_tool = new Vector4(0, 0, -1, 1);
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
                        hipStatus.color = Color.yellow;
                        TCPCommunication.message = "";
                    }



                    // INITIALISATION: set relative plan location, otherwise is origin and unit
                    else if (dataSplit.Length == 14)
                    {
                        // set guidance position: surgical plan in femur coordinate
                        localstart.x = float.Parse(dataSplit[0]);
                        localstart.y = float.Parse(dataSplit[1]);
                        localstart.z = float.Parse(dataSplit[2]);
                        localend.x = float.Parse(dataSplit[3]);
                        localend.y = float.Parse(dataSplit[4]);
                        localend.z = float.Parse(dataSplit[5]);

                        localstart_tool.x = float.Parse(dataSplit[6]);
                        localstart_tool.y = float.Parse(dataSplit[7]);
                        localstart_tool.z = float.Parse(dataSplit[8]);
                        localend_tool.x = float.Parse(dataSplit[9]);
                        localend_tool.y = float.Parse(dataSplit[10]);
                        localend_tool.z = float.Parse(dataSplit[11]);

                        thres_pos = float.Parse(dataSplit[12]);
                        thres_ang = float.Parse(dataSplit[13]);

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
                        Vector3 angles = poseWUnity.rotation.eulerAngles;
                        Quaternion quats = Quaternion.LookRotation(poseWUnity.GetColumn(2), poseWUnity.GetColumn(1));

                        if (set_base)// pulse like
                        {
                            set_base = false;
                            base_exist = true;
                            trans_base = trans;
                            angles_base = angles;

                            trans_last = trans;
                            quats_last = quats;

                            myText.text = myText.text + "base set!\n";
                            kfStatus.color = Color.green;

                        }

                        // has to be set first
                        if (base_exist)
                        {
                            // outlier rejection by euler angles
                            if (Vector3.Distance(trans, trans_base) < thres_pos && Math.Abs(angles.x - angles_base.x) < thres_ang && Math.Abs(angles.y - angles_base.y) < thres_ang && Math.Abs(angles.z - angles_base.z) < thres_ang)
                            {
                                // then this is inlier, push into lerp
                                Vector3 trans_lerp = Vector3.Lerp(trans_last, trans, 0.6f);
                                Quaternion quats_lerp = Quaternion.Lerp(quats_last, quats, 0.6f);
                                Matrix4x4 poseWUnity_kf = Matrix4x4.Rotate(quats_lerp);
                                poseWUnity_kf.SetColumn(3, trans_lerp);
                                trans_last = trans_lerp;  // update last values
                                quats_last = quats_lerp;


                                myText.text = trans.ToString("F5") + quats.ToString("F5") + "\n" + trans_lerp.ToString("F5") + quats_lerp.ToString("F5") + "\n";

                                // femur overlay
                                femurObj.transform.position = poseWUnity_kf.GetColumn(3);  // entry point
                                femurObj.transform.rotation = poseWUnity_kf.rotation;  // entry point

                                // plan overlay by start and end
                                planstart = poseWUnity_kf * localstart;
                                planend = poseWUnity_kf * localend;

                                plan.transform.localScale = new Vector3(1, 1, Vector3.Distance(planstart, planend) / 2);
                                plan.transform.position = planstart;        // place bond here
                                plan.transform.LookAt(planend);

                                kfStatus.color = Color.green;


                            }
                            // else, world locked, indicate status
                            else
                            {
                                kfStatus.color = Color.blue;
                                myText.text = myText.text + trans_base.ToString("F5") + angles_base.ToString("F5") + " but " + trans.ToString("F5") + angles.ToString("F5") + "\n";

                            }

                            // only track when base is locked: tool
                            Matrix4x4 tool_W = tooler.GetLatestToolPose();
                            Vector3 trans_tool = tool_W.GetColumn(3);
                            Quaternion quats_tool = Quaternion.LookRotation(tool_W.GetColumn(2), tool_W.GetColumn(1));
                            if (trans_last_tool == Vector3.zero)
                            {
                                trans_last_tool = trans_tool;
                                quats_last_tool = quats_tool;
                            }
                            // then push into lerp
                            Vector3 trans_lerp_tool = Vector3.Lerp(trans_last_tool, trans_tool, 0.8f);
                            Quaternion quats_lerp_tool = Quaternion.Lerp(quats_last_tool, quats_tool, 0.8f);
                            Matrix4x4 tool_W_lerp = Matrix4x4.Rotate(quats_lerp_tool);
                            tool_W_lerp.SetColumn(3, trans_lerp_tool);
                            trans_last_tool = trans_lerp_tool;
                            quats_last_tool = quats_lerp_tool;

                            // tool overlay by start and end
                            Vector3 toolstart = tool_W_lerp * localstart_tool;
                            Vector3 toolend = tool_W_lerp * localend_tool;

                            tool.transform.localScale = new Vector3(1, 1, Vector3.Distance(toolstart, toolend) / 2);
                            tool.transform.position = toolstart;        // place bond here
                            tool.transform.LookAt(toolend);

                            // calculate difference
                            float pos_err = Vector3.Distance(toolstart, planstart) * 1000;
                            float dir_err = Vector3.Angle((toolend - toolstart), (planend - planstart));
                            ErrInfo.text = pos_err.ToString("F2") + " mm, " + dir_err.ToString("F2") + "\u00B0";

                            if (pos_err < 3)
                                plan.transform.Find("head").GetComponent<Renderer>().material.SetColor("_Color", Color.green);
                            else
                                plan.transform.Find("head").GetComponent<Renderer>().material.SetColor("_Color", Color.red);

                            if (dir_err < 3)
                                plan.transform.Find("body").GetComponent<Renderer>().material.SetColor("_Color", Color.green);
                            else
                                plan.transform.Find("body").GetComponent<Renderer>().material.SetColor("_Color", Color.red);

                        }
                        else
                        {
                            // femur overlay, inidcated by red kf light 
                            femurObj.transform.position = poseWUnity.GetColumn(3);  // entry point
                            femurObj.transform.rotation = poseWUnity.rotation;  // entry point

                            // plan: related to the initalised plan
                            Vector3 planstart = poseWUnity * localstart;
                            Vector3 planend = poseWUnity * localend;
                            plan.transform.localScale = new Vector3(1, 1, Vector3.Distance(planstart, planend) / 2);
                            plan.transform.position = planstart;        // place bond here
                            plan.transform.LookAt(planend);
                        }

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
                kfStatus.color = Color.red;
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

            hipStatus.color = Color.green;

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

            hipStatus.color = Color.red;
            kfStatus.color = Color.red;

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



