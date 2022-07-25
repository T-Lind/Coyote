package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

/**
 * A Kalman Filter for 1D data.
 * It can be used for multiple dimensioned data, however a separate
 * object must be created for each dimension.
 * Created for Victorian Voltage (FTC team 7797) 2022-2023 FIRST Tech Challenge Season/Preseason ONLY
 * 9527 Rogue Resistance or any other team may not use this code without permission from the author.
 *
 * Use this object in a runner like any other.
 *
 * @author Tiernan Lindauer
 */
public class KalmanFilter {


    /**
     * Vector A
     */
    private double A = 1;
    /**
     * Vector B
     */
    private double B = 0;
    /**
     * Vector C
     */
    private double C = 1;

    /**
     * Process noise
     */
    private double R;
    /**
     * Measurement noise
     */
    private double Q;

    /**
     * Covariance variable
     */
    private double cov = Double.NaN;

    /**
     * State variable
     */
    private double x = Double.NaN;

    /**
     * Kalman Filter for driving motors during pathing
     */
    public KalmanFilter() {
        this.R = Robot.R;
        this.Q = Robot.Q;

        this.C = Robot.C;
        this.B = Robot.B;
        this.A = Robot.A;
    }

    /**
     * Instead of specifying a deviceCode, make a custom Kalman Filter.
     * @param R is process noise
     * @param Q is measurement noise
     * @param A is state vector
     * @param B is control vector
     * @param C is measurement vector
     */
    public KalmanFilter(double R, double Q, double A, double B , double C){
        this.R = R;
        this.Q = Q;
        this.A = A;
        this.B = B;
        this.C = C;

        this.cov = Double.NaN;
        this.x = Double.NaN; // estimated signal without noise
    }

    /**
     * Only specify noise
     * @param R is process noise
     * @param Q is measurement noise
     */
    // R is process noise, Q is measurement noise. No specified state/control/measurement vectors, set to default 1,0,1
    public KalmanFilter(double R, double Q){
        this.R = R;
        this.Q = Q;

    }

    /**
     * Feed a new value into the Kalman filter and return what the predicted state is.
     * @param measurement the measured value
     * @param u is the controlled input value
     * @return the predicted result.
     * Postcondition: the appropriate filtered value has been returned
     */
    // Filter a measurement: measured value is measurement, controlled input value is u.
    public final double filter(double measurement, double u){

        if (Double.isNaN(this.x)) {
            this.x = (1 / this.C) * measurement;
            this.cov = (1 / this.C) * this.Q * (1 / this.C);
        }else {
            double predX = (this.A * this.x) + (this.B * u);
            double predCov = ((this.A * this.cov) * this.A) + this.R;

            // Kalman gain
            double K = predCov * this.C * (1 / ((this.C * predCov * this.C) + this.Q));

            // Correction
            this.x = predX + K * (measurement - (this.C * predX));
            this.cov = predCov - (K * this.C * predCov);
        }
        return this.x;
    }

    /**
     * Feed a new value into the Kalman filter and return what the predicted state is.
     * @param measurement the measured value
     * @return the predicted result.
     * Postcondition: the appropriate filtered value has been returned
     */
    // Filter a measurement taken
    public final double filter(double measurement){
        double u = 0;
        if (Double.isNaN(this.x)) {
            this.x = (1 / this.C) * measurement;
            this.cov = (1 / this.C) * this.Q * (1 / this.C);
        }else {
            double predX = (this.A * this.x) + (this.B * u);
            double predCov = ((this.A * this.cov) * this.A) + this.R;

            // Kalman gain
            double K = predCov * this.C * (1 / ((this.C * predCov * this.C) + this.Q));

            // Correction
            this.x = predX + K * (measurement - (this.C * predX));
            this.cov = predCov - (K * this.C * predCov);
        }
        return this.x;
    }


    /**
     * Return the last measurement taken.
     * @return the last measurement
     * Postcondition: returns the last measurement accurately
     */
    // Return the last measurement taken
    public final double lastMeasurement(){
        return this.x;
    }

    /**
     * Set the measurement noise
     * @param noise the measurement noise.
     * Postcondition: sets the measurement noise accurately
     */
    // Set measurement noise
    public final void setMeasurementNoise(double noise){
        this.Q = noise;
    }

    /**
     * Set the process noise
     * @param noise the process noise.
     * Postcondition: sets the process noise accurately
     */
    public final void setProcessNoise(double noise){
        this.R = noise;
    }
}
