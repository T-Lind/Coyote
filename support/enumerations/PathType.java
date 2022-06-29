package org.firstinspires.ftc.teamcode.auto.support.enumerations;

/**
 * Define what type of path is being used
 */
public enum PathType {
    /**
     * The path to move in a line.
     */
    LINE,
    /**
     *  The path to move in a spline (SplinePath).
     */
    TURN,
    /**
     * The path to move in a turn.
     */
    SPLINE,

    /**
     * Follow a spline at a constant heading (only used for differential swerve)
     */
    CONSTANTSPLINE
}