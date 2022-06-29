package org.firstinspires.ftc.teamcode.auto.support.enumerations;

/**
 * What type of external device is being used which needs some form of control (ex. Kalman filter or PID loop)
 *
 */
public enum PeripheralType {
    /**
     * Default set to none.
     */
    NONE,
    /**
     * For driving straight.
     */
    DRIVETRAIN_MOTOR_STRAIGHT,
    /**
     * Kalman Filter only - used for distance sensors.
     */
    DISTANCE_SENSOR,
    /**
     * For turning.
     */
    DRIVETRAIN_MOTOR_TURN
}
