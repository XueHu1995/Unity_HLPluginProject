using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class BICP_CALI : MonoBehaviour
{
    //public Text myText;
    //public Text myText2;

    Matrix4x4 P_l = Matrix4x4.identity;
    Matrix4x4 P_r = Matrix4x4.identity;

    Matrix4x4 UQ = Matrix4x4.identity;
    Camera main;
    // Start is called before the first frame update
    void Start()
    {
        P_r = Camera.main.GetStereoProjectionMatrix(Camera.StereoscopicEye.Right);
        P_l = Camera.main.GetStereoProjectionMatrix(Camera.StereoscopicEye.Left);
        main = Camera.main;

    }

    // Update is called once per frame
    void Update()
    {
        //myText.text = Camera.main.GetStereoProjectionMatrix(Camera.StereoscopicEye.Left).ToString("F5");
    }

    public void MoveLeft()
    {
        UQ.m03 = UQ.m03 - 0.001f;


        Matrix4x4 P_set_l = P_l * UQ;
        Matrix4x4 P_set_r = P_r * UQ;

        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Right, P_set_r);
        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Left, P_set_l);
    }
    public void MoveRight()
    {
        UQ.m03 = UQ.m03 + 0.001f;


        Matrix4x4 P_set_l = P_l * UQ;
        Matrix4x4 P_set_r = P_r * UQ;

        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Right, P_set_r);
        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Left, P_set_l);

    }
    public void MoveUp()
    {
        UQ.m13 = UQ.m13 + 0.001f;


        Matrix4x4 P_set_l = P_l * UQ;
        Matrix4x4 P_set_r = P_r * UQ;

        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Right, P_set_r);
        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Left, P_set_l);
    }
    public void MoveDown()
    {
        UQ.m13 = UQ.m13 - 0.001f;


        Matrix4x4 P_set_l = P_l * UQ;
        Matrix4x4 P_set_r = P_r * UQ;

        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Right, P_set_r);
        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Left, P_set_l);
    }
    public void MoveFront()
    {
        UQ.m23 = UQ.m23 - 0.003f;


        Matrix4x4 P_set_l = P_l * UQ;
        Matrix4x4 P_set_r = P_r * UQ;

        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Right, P_set_r);
        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Left, P_set_l);
    }
    public void MoveBack()
    {
        UQ.m23 = UQ.m23 + 0.003f;


        Matrix4x4 P_set_l = P_l * UQ;
        Matrix4x4 P_set_r = P_r * UQ;

        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Right, P_set_r);
        main.SetStereoProjectionMatrix(Camera.StereoscopicEye.Left, P_set_l);
    }
    public void ResetProj()
    {
        Camera.main.ResetStereoProjectionMatrices();
        //myText2.text = "reset!";
    }

}
