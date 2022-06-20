package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.DrivetrainSymmetry;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

/**
 * Program to turn a specified angle.
 * @author Tiernan Lindauer
 * Uses the Line Class and cleverly overrides the left motor velocity.
 */
public class Turn extends Line{

    /**
     * Velocity coefficients for driving an asymmetrical drivetrain
     */
    protected static final int[][] asymmetricalDriveCoefficientLookup = {
            {1, -1}, // LEFT CCW, LEFT CW
            {1, -1} // RIGHT CCW, RIGHT CW
    };

    /**
     * Velocity coefficients for driving a symmetrical drivetrain
     */
    protected static final int[][] symmetricalDriveCoefficientLookup = {
            {-1, 1}, // LEFT CCW, LEFT CW
            {1, -1} // RIGHT CCW, RIGHT CW
    };

    /**
     * @param angleToTurn is the angle traveled (degrees)
     * @param maxVelocity is the maximum velocity to travel at
     */
    public Turn(double angleToTurn, double maxVelocity) {
        super((3.14159*angleToTurn*Path.getTrackWidth())/360, maxVelocity);
    }

    /**
     * @param angleToTurn is the angle traveled (degrees)
     * @param maxVelocity is the maximum velocity to travel at
     * @param reversed is whether or not the drivetrain is reversed
     */
    public Turn(double angleToTurn, double maxVelocity, boolean reversed) {
        super((3.14159*angleToTurn*Path.getTrackWidth())/360, maxVelocity, reversed);
    }

    /**
     * Get the left linear velocity
     * @param currentTime the current time
     * @return left linear velocity
     */
    @Override
    public final double getLeftVelocity(double currentTime){
        return getVelocity(currentTime)*velocityLookupTable(asymmetricalDriveCoefficientLookup, symmetricalDriveCoefficientLookup, 0);
    }

    /**
     * Get the right side velocity
     * @param currentTime the current time
     * @return the right side velocity so as to turn a certain amount.
     * Precondition:  current time is not less than zero
     * Postcondition: the accurate right velocity is returned
     */
    @Override
    public final double getRightVelocity(double currentTime){
        if(currentTime < 0)
            throw new RuntimeException("currentTime in Turn.getRightVelocity() must be greater than or equal to zero");

        return getVelocity(currentTime)*velocityLookupTable(asymmetricalDriveCoefficientLookup, symmetricalDriveCoefficientLookup, 1);
    }

    /**
     * Get what type of path this is. Useful for debugging
     * @return The type of path, in this case a turn.
     * Postcondition: the correct type is returned
     */
    @Override
    public final PathType getType(){
        return PathType.TURN;
    }
}
