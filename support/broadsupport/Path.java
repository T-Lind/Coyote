package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.BuildStatus;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.Direction;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.DrivetrainSymmetry;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

/**
 * Parent class for paths that solely are composed of time to wheel velocity functions.
 * Essentially acts as a piecewise function over time for both the left and right velocity.
 */
public abstract class Path {
    /**
     * The time it takes this path to execute
     */
    private double executeTime = 0;

    /**
     * Whether or not this path has finished running
     */
    private boolean completed = false;

    /**
     * The build status of this path instantiation
     */
    private BuildStatus construction = BuildStatus.UNBUILT;

    /**
     * Whether the robot is moving forward or backward in this path
     */
    private Direction moveState = Direction.FORWARD;

    /**
     * Velocity coefficients for driving an asymmetrical drivetrain
     */
    protected static final int[][] asymmetricalDriveCoefficientLookup = {
            {-1, 1}, // LEFT FORWARD, LEFT REVERSE
            {1, -1} // RIGHT FORWARD, RIGHT REVERSE
    };

    /**
     * Velocity coefficients for driving a symmetrical drivetrain
     */
    protected static final int[][] symmetricalDriveCoefficientLookup = {
            {1, -1}, // LEFT FORWARD, LEFT REVERSE
            {1, -1} // RIGHT FORWARD, RIGHT REVERSE
    };

    /**
     * What type of drivetrain is being used
     */
    private static DrivetrainSymmetry symmetryState;

    /**
     * Track width of the robot
     */
    private static double trackWidth;


    public void setMoveState(Direction moveState){
        this.moveState = moveState;
    }
    public Direction getMoveState(){
        return moveState;
    }

    public static void setSymmetryState(DrivetrainSymmetry symmetryState){
        Path.symmetryState = symmetryState;
    }
    public static DrivetrainSymmetry getSymmetryState(){
        return symmetryState;
    }

    public static void setTrackWidth(double trackWidth){
        Path.trackWidth = trackWidth;
    }
    public static double getTrackWidth(){
        return trackWidth;
    }

    /**
     * Method returning the type of path this specific path is
     *
     * @return the correct type of path.
     */
    abstract public PathType getType();

    /**
     * Meant to compute the trajectory and essentially turn it into a piecewise function.
     * This method is supposed to be overriden.
     *
     * Precondition:  the construction variable has been instantiated
     * Postcondition: the path is set to a built status
     */
    public abstract void build();

    /**
     * Set the execute time - time it takes to follow this specific Path.
     *
     * @param executeTime is the new execute time.
     * Precondition:  execute time is greater than zero
     * Postcondition: the execute time has been set
     */
    public final void setExecuteTime(double executeTime) {
        if(executeTime < 0)
            throw new RuntimeException("The execute time in Path.setExecuteTime(...) must be greater than zero!");
        this.executeTime = executeTime;
    }

    /**
     * Get the current execute time - time it takes to follow this specific Path.
     *
     * @return the execute time.
     */
    public double getExecuteTime() {
        if(executeTime < 0)
            throw new RuntimeException("The execute time in Path.getExecuteTime(...) must be greater than zero!");
        return executeTime;
    }

    /**
     * Set the completed status - not overriden.
     *
     * @param completionStatus is the new status of the path (should always be assigned as true).
     * Precondition:  the completion status parameter accurately reflects the state of the robot
     * Postcondition: the completed variable is set
     */
    public void setCompleted(boolean completionStatus) {
        completed = completionStatus;
    }

    /**
     * Get the completed status - not overriden.
     *
     * @return the completed status.
     * Precondition:  the completed status has been accurately updated
     * Postcondition: the status of completion has been returned
     */
    public final boolean getCompleted() {
        return completed;
    }

    /**
     * Set the built boolean.
     *
     * @param isBuilt is the build status.
     * Precondition:  the isBuilt parameter accurately reflects the status of the path
     * Postcondition: the construction variable is accurately set
     */
    public final void setBuilt(boolean isBuilt) {
        if (isBuilt)
            construction = BuildStatus.BUILT;
        else
            construction = BuildStatus.UNBUILT;
    }

    /**
     * Get the build status of this Path.
     *
     * @return the built variable (build status).
     * Precondition:  the build status has been accurately updated
     * Postcondition: the accurate build status has been returned
     */
    public final boolean getBuilt() {
        return construction == BuildStatus.BUILT;
    }

    /**
     * Get the left velocity - meant to be overridden, defaults to zero speed.
     * Precondition: currentTime is greater than zero
     *
     * @param currentTime is the current time into this specific Path.
     * @return default - zero
     */
    public double getLeftVelocity(double currentTime) {
        if(currentTime < 0)
            throw new RuntimeException("The time to get the left velocity for in Path.getLeftVelocity(...) must not be less than zero!");
        return 0;
    }

    /**
     * Get the right velocity - meant to be overridden, defaults to zero speed.
     * Precondition: currentTime is greater than zero
     *
     * @param currentTime is the current time into this specific Path.
     * @return default - zero
     */
    public double getRightVelocity(double currentTime) {
        if(currentTime < 0)
            throw new RuntimeException("The time to get the right velocity for in Path.getRightVelocity(...) must not be less than zero!");
        return 0;
    }

    /**
     * Get the left angle - meant to be overridden, defaults to zero.
     * Precondition: currentTime is greater than zero
     *
     * @param currentTime is the current time into this specific Paths
     * @return default - zero
     */
    public double getLeftAngularVelocity(double currentTime) {
        if(currentTime < 0)
            throw new RuntimeException("The time to get the left velocity for in Path.getLeftAngle(...) must not be less than zero!");
        return 0;
    }

    /**
     * Get the right angle - meant to be overridden, defaults to zero.
     * Precondition: currentTime is greater than zero
     *
     * @param currentTime is the current time into this specific Path.
     * @return default - zero
     */
    public double getRightAngularVelocity(double currentTime) {
        if(currentTime < 0)
            throw new RuntimeException("The time to get the right velocity for in Path.getRightAngle(...) must not be less than zero!");
        return 0;
    }

    /**
     * Method to convert distance for a simple 2/4/6 wheel drivetrain - NOT diffy swerve!
     * @param wheelRadius is the radius, NOT DIAMETER, of the wheel
     * @param velocity    is the linear velocity in m/s
     * @return the angular velocity in rad/s
     */
    public static double convertForStandardDrivetrain(double wheelRadius, double velocity) {
        if(wheelRadius < 0 || velocity < 0)
            throw new RuntimeException("Wheel radius and velocity must be greater than zero in Path.convert(...)");
        velocity /= wheelRadius; // convert to angular velocity by radius
        velocity /= (2 * 3.14159);
        return velocity;
    }

    /**
     * Use the provided lookup tables to identify the velocity easier
     * @param asymmetrical the asymmetrical lookup table
     * @param symmetrical the symmetrical lookup table
     * @param sideIndex the side index - 0 is left, 1 is right
     * @return the correct velocity for the motor
     * Precondition: current time is not less than zero, arrays are 2x2 each, side index is 0 or 1
     * Postcondition: accurate velocity is returned
     */
    protected final double velocityLookupTable(int[][] asymmetrical, int[][] symmetrical, int sideIndex){
        if(asymmetrical.length != 2 || symmetrical.length != 2 || sideIndex < 0 || sideIndex > 1)
            throw new RuntimeException("Invalid parameter in Line.velocityLookupTable(...)!");

        int lookupIndex = 0;
        if(getMoveState() == Direction.REVERSE)
            lookupIndex = 1;

        int coeff;

        if(symmetryState == DrivetrainSymmetry.ASYMMETRICAL)
            coeff = asymmetrical[sideIndex][lookupIndex];
        else
            coeff = symmetrical[sideIndex][lookupIndex];

        return coeff;
    }
}
