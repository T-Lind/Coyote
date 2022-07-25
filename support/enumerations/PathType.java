package org.firstinspires.ftc.teamcode.auto.support.enumerations;

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
     * Move in a spline but at a constant heading on a differential swerve
     */
    CONSTANTSPLINE,

    /**
     * Turn only differential swerve pods
     */
    PODTURN
}