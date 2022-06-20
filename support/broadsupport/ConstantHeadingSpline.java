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
    @Override
    public final double getLeftVelocity(double currentTime) {
       double leftV = super.getLeftVelocity(currentTime);
       double rightV = super.getRightVelocity(currentTime);

       if(leftV < 0)
           return -(Math.abs(leftV-rightV));
        return (Math.abs(leftV-rightV));
    }

    @Override
    public final double getRightVelocity(double currentTime){
        return getLeftVelocity(currentTime);
    }

    @Override
    public final double getLeftAngularVelocity(double currentTime){
        return 2*(getVelocity(currentTime)-super.getLeftVelocity(currentTime));
    }

    @Override
    public final double getRightAngularVelocity(double currentTime){
        return getLeftAngularVelocity(currentTime);
    }

    /**
     * Method returning the type of path this specific path is
     *
     * @return the correct type of path.
     */
    @Override
    public final PathType getType() {
        return PathType.CONSTANTSPLINE;
    }
}
