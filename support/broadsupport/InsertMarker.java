package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

/**
 * An interface to execute different parts of code at different times in the trajectory.
 * Useful for running arms, motors, etc. while in movement.
 * @author Tiernan Lindauer
 */
public interface InsertMarker {
    /**
     * execute what the lambda expression says immediately
     */
    void execute();
}
