package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

public class ConstantHeadingSpline extends SplinePath{
    /**
     * Constructor for SplinePath with reversed boolean.
     * Each static array in v is 3 long, size of r and v is equal
     * Convention: positive arc length is CCW, negative is CW
     *
     * @param velocity         maximum linear velocity (m/s)
     * @param accelerationTime acceleration and deceleration time (s)
     * @param radii            is the radii of the arcs in the spline
     * @param arcLengths       are the arc lengths travelled in each corresponding radii.
     */
    public ConstantHeadingSpline(double velocity, double accelerationTime, double[] radii, double[] arcLengths) {
        super(velocity, accelerationTime, radii, arcLengths);
    }

    /**
     * Method to get the left linear velocity (will be the same as the right)
     * for a constant heading spline.
     * @param currentTime is the current time into the spline.
     * @return the left linear velocity for this spline.
     */
    @Override
    public final double getLeftVelocity(double currentTime) {
       double leftV = super.getLeftVelocity(currentTime);
       double rightV = super.getRightVelocity(currentTime);

       return Math.min(leftV, rightV);
    }

    /**
     * Method to get the right linear velocity (will be the same as the left)
     * for a constant heading spline. References the left method.
     * @param currentTime is the current time into the spline.
     * @return the right linear velocity for this spline.
     */
    @Override
    public final double getRightVelocity(double currentTime){
        return getLeftVelocity(currentTime);
    }

    /**
     * Method to get the left angular velocity (will be the same as the right)
     * for a constant heading spline.
     * @param currentTime is the current time into the spline.
     * @return the left angular velocity for this spline.
     */
    @Override
    public final double getLeftAngularVelocity(double currentTime){
        return 2*(getVelocity(currentTime)-super.getLeftVelocity(currentTime));
    }

    /**
     * Method to get the right angular velocity (will be the same as the right)
     * for a constant heading spline.
     * @param currentTime is the current time into the spline.
     * @return the right angular velocity for this spline.
     */
    @Override
    public final double getRightAngularVelocity(double currentTime){
        return getLeftAngularVelocity(currentTime);
    }

    /**
     * Method returning the type of path this specific path is
     * @return the correct type of path.
     */
    @Override
    public final PathType getType() {
        return PathType.CONSTANTSPLINE;
    }
}
