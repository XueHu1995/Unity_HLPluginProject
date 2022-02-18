using LightweightMatrixCSharp;

/// <summary>
/// Kalman filter implementation
/// </summary>
public class KalmanFilter
{
    public Matrix P { get; private set; }
    public Matrix Q { get; private set; }

    public Matrix X { get; private set; }
    public Matrix H { get; private set; }
    public Matrix A { get; private set; }

    public Matrix R { get; private set; }

    public bool initialised = false;
    public KalmanFilter(Matrix errorCovPostMat, Matrix measurementNoiseCovMat, Matrix processNoiseCovMat, Matrix measurementMat, Matrix transitionMat)
    {
        P = errorCovPostMat;
        Q = processNoiseCovMat;
        R = measurementNoiseCovMat;
        H = measurementMat;
        A = transitionMat;
    }

    public void SetInitialState(Matrix initialState)
    {
        X = initialState;
        initialised = true;
    }


    public void SetCovMat(Matrix processNoiseCovMat, Matrix measurementNoiseCovMat)
    {
        Q = processNoiseCovMat;
        R = measurementNoiseCovMat;
    }

    public void Predict()
    {
        // Update state estimate using previous state
        X = A * X;

        // Update state covariance Matrix (pre) errorCovPre
        P = (A * P * Matrix.Transpose(A)) + Q;
    }

    public void Correct(Matrix z)
    {
        // Calculate Kalman Gain
        var K = P * Matrix.Transpose(H) * ((H * P * Matrix.Transpose(H)) + R).Invert();

        // Update state estimate statePost
        X = X + (K * (z - (H * X)));

        // Update state covariance Matrix (post) errorCovPost
        var I = Matrix.IdentityMatrix(X.rows, X.rows);
        P = (I - K * H) * P;
    }
}
