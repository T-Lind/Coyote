package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.Direction;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.DrivetrainSymmetry;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

/**
 * Creates a list of velocities for the wheels on a robot to move at.
 * created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 *
 */

public class Line extends Path {
    // Variables to model the type of line this path should follow

    /**
     * Distance this path should go
     */
    private final double distance;

    /**
     * Maximum velocity the robot should reach
     */
    private final double maxVelocity;

    /**
     * Time it takes to complete this path
     */
    private double executeTime;


    /**
     * Constructor for this Line object. Here it make an assumption since it has not been given
     * enough data - sets the drivtrain type to asymmetrical
     * @param distance is the distance traveled
     * @param maxVelocity is the maximum velocity to travel at
     */
    public Line(double distance, double maxVelocity){
        if(maxVelocity == 0 || distance == 0)
            throw new RuntimeException("The velocity or distance must not be equal to zero in Line.Line(...)!");
        this.distance = distance;
        this.maxVelocity = maxVelocity;

        executeTime = 0;
    }
    /**
     * Constructor for this Line object. Here it make an assumption since it has not been given
     * enough data - sets the drivtrain type to asymmetrical
     * @param distance is the distance traveled
     * @param maxVelocity is the maximum velocity to travel at
     * @param reversed is whether or not the drivetrain is reversed
     */
    public Line(double distance, double maxVelocity, boolean reversed){
        if(maxVelocity == 0 || distance == 0)
            throw new RuntimeException("The velocity or distance must not be equal to zero in Line.Line(...)!");
        this.distance = distance;
        this.maxVelocity = maxVelocity;

        executeTime = 0;

        if(!reversed)
            setMoveState(Direction.FORWARD);
        else
            setMoveState(Direction.REVERSE);
    }


    /**
     * Build the line trajectory.
     * Postcondition: this path has been built successfully
     */
    @Override
    public final void build(){
        executeTime = 3.14159265*Math.abs(distance)/(2*maxVelocity);
        setBuilt(true);
    }

    /**
     * Get the time it takes to run the path
     * @return the execution time
     */
    @Override
    public final double getExecuteTime(){
        return executeTime;
    }

    /**
     * Get the linear velocity of the entire bot
     * @param t the current time
     * @return the linear velocity of the bot
     */
    protected double getVelocity(double t){
        if(t < executeTime){
            if(distance > 0)
                return maxVelocity*Math.sin(3.1415925*t/ executeTime);
            return -1*maxVelocity*Math.sin(3.1415925*t/ executeTime);
        }
        else{
            this.setCompleted(true);
            return 0;
        }
    }

    /**
     * Get the left linear velocity
     * @param currentTime the current time
     * @return left linear velocity
     */
    @Override
    public double getLeftVelocity(double currentTime){
        return getVelocity(currentTime)*velocityLookupTable(asymmetricalDriveCoefficientLookup, symmetricalDriveCoefficientLookup, 0);
    }
    /**
     * Get the right linear velocity
     * @param currentTime the current time
     * @return right linear velocity
     */
    @Override
    public double getRightVelocity(double currentTime){
        return getVelocity(currentTime)*velocityLookupTable(asymmetricalDriveCoefficientLookup, symmetricalDriveCoefficientLookup, 1);
    }

    /**
     * Get what type of path this is. Useful for debugging
     * @return The type of path, in this case a line.
     */
    @Override
    public PathType getType(){
        return PathType.LINE;
    }
}