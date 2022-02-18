using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*! 
*  \brief     Matrix4x4 utility helper
*  \details   Helper matrix utility Functions handling UnityEngine's standard Matrix4x4 type
*  \author    Hisham Iqbal
*  \date      2021
*/
public class MatrixUtilities
{
    public enum MatrixEntryOrder
    {
        RowMajor = 0,
        ColumnMajor = 1
    }

    public enum MatrixUnits
    {
        mm = 0,
        m = 1
    }

    public enum Direction
    {
        x,
        y,
        z
    }

    /// <summary>
    /// Tries to parse a double array into a Matrix4x4
    /// </summary>
    /// <param name="vals">Raw double values for matrix, should be 16 elements</param>
    /// <param name="order">Enum describing if elements are in row/column major order</param>
    /// <param name="units">Enum describing units of matrix elements</param>
    /// <returns>Formatted Matrix4x4 filled with values</returns>
    public static Matrix4x4 FillMatrixWithDoubles(double[] vals, MatrixEntryOrder order, MatrixUnits units)
    {
        if (vals.Length < 16) { return Matrix4x4.identity; }

        Matrix4x4 returnMat = Matrix4x4.zero;

        Vector4 col1 = new Vector4((float)vals[0], (float)vals[1], (float)vals[2], (float)vals[3]);
        Vector4 col2 = new Vector4((float)vals[4], (float)vals[5], (float)vals[6], (float)vals[7]);
        Vector4 col3 = new Vector4((float)vals[8], (float)vals[9], (float)vals[10], (float)vals[11]);
        Vector4 col4 = new Vector4((float)vals[12], (float)vals[13], (float)vals[14], (float)vals[15]);

        returnMat.SetColumn(0, col1); returnMat.SetColumn(1, col2);
        returnMat.SetColumn(2, col3); returnMat.SetColumn(3, col4);

        // check if you need to transpose and correctly orient the matrix
        if (order == MatrixEntryOrder.ColumnMajor)
        {
            // no transpose do nothing
        }
        else if (order == MatrixEntryOrder.RowMajor)
        {
            returnMat = returnMat.transpose;
        }
        else
        {
            Debug.Log("Invalid matrix order inputted");
            return Matrix4x4.zero;
        }

        // check and handle units to see if scaling is required

        if (units == MatrixUnits.m) { return returnMat; } //no scaling required, input was already in metres

        else if (units == MatrixUnits.mm)
        {
            returnMat.SetColumn(3, new Vector4(returnMat[0, 3] / 1000f, returnMat[1, 3] / 1000f, returnMat[2, 3] / 1000f, 1f));
            return returnMat;
        }

        else { Debug.Log("Invalid MatrixUnits enum value supplied"); return Matrix4x4.zero; }
    }

    public static Matrix4x4 FillMatrixWithFloats(float[] vals, MatrixEntryOrder order, MatrixUnits units)
    {
        if (vals.Length < 16) { return Matrix4x4.identity; }

        Matrix4x4 returnMat = Matrix4x4.zero;

        Vector4 col1 = new Vector4((float)vals[0], (float)vals[1], (float)vals[2], (float)vals[3]);
        Vector4 col2 = new Vector4((float)vals[4], (float)vals[5], (float)vals[6], (float)vals[7]);
        Vector4 col3 = new Vector4((float)vals[8], (float)vals[9], (float)vals[10], (float)vals[11]);
        Vector4 col4 = new Vector4((float)vals[12], (float)vals[13], (float)vals[14], (float)vals[15]);

        returnMat.SetColumn(0, col1); returnMat.SetColumn(1, col2);
        returnMat.SetColumn(2, col3); returnMat.SetColumn(3, col4);

        // check if you need to transpose and correctly orient the matrix
        if (order == MatrixEntryOrder.ColumnMajor)
        {
            // no transpose do nothing
        }
        else if (order == MatrixEntryOrder.RowMajor)
        {
            returnMat = returnMat.transpose;
        }
        else
        {
            Debug.Log("Invalid matrix order inputted");
            return Matrix4x4.zero;
        }

        // check and handle units to see if scaling is required

        if (units == MatrixUnits.m) { return returnMat; } //no scaling required, input was already in metres

        else if (units == MatrixUnits.mm)
        {
            returnMat.SetColumn(3, new Vector4(returnMat[0, 3] / 1000f, returnMat[1, 3] / 1000f, returnMat[2, 3] / 1000f, 1f));
            return returnMat;
        }

        else { Debug.Log("Invalid MatrixUnits enum value supplied"); return Matrix4x4.zero; }
    }


    /// <summary>
    /// Helper function to convert between left/right handed matrices, achieved by inverting one dimension in the matrix
    /// </summary>
    /// <param name="_m">Input matrix</param>
    /// <param name="dir">Direction along which to invert</param>
    /// <returns>Output matrix with swapped handedness</returns>
    public static Matrix4x4 ReturnSwapHandedMatrix(Matrix4x4 _m, Direction dir)
    {
        int xFactor = 1, yFactor = 1, zFactor = 1;

        switch (dir)
        {
            case Direction.x: xFactor = -1; break;
            case Direction.y: yFactor = -1; break;
            case Direction.z: zFactor = -1; break;
        }

        /*[ xx yx zx x
            xy yy zy y
            xz yz zz z ]*/

        // deal with rotation matrix ???
        _m[0, 1] *= (yFactor * xFactor); _m[0, 2] *= (zFactor * xFactor);
        _m[1, 0] *= (xFactor * yFactor); _m[1, 2] *= (zFactor * yFactor);
        _m[2, 0] *= (xFactor * zFactor); _m[2, 1] *= (yFactor * zFactor); 

        // deal with translation vector
        _m[0, 3] *= xFactor; _m[1, 3] *= yFactor; _m[2, 3] *= zFactor;
        return _m;
    }

}
