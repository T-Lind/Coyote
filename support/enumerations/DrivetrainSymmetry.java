package org.firstinspires.ftc.teamcode.auto.support.enumerations;

/**
 * Enumeration to specify the state whether the drivetrain motors are symmetrical or asymmetrical.
 * For most drivetrain designs except differential swerve it will be asymmetrical.
 */
public enum DrivetrainSymmetry {
    /**
     * The drivetrain is reflected over the center (intake) axis.
     */
    SYMMETRICAL,
    /**
     * The drivetrain is not reflectd over the center axis. Typically only applies to differential swerves, but not all.
     */
    ASYMMETRICAL
}
