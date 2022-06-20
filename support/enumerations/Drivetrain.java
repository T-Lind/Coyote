package org.firstinspires.ftc.teamcode.auto.support.enumerations;

public enum Drivetrain {
    /**
     * No drivetrain. Used as default.
     */
    NONE,
    /**
     * For use with two motor robots (aka pushbots).
     */
    TWOWD,
    /**
     * Four wheel drive type drivetrain. Could be used with mecanum but no mecanum specific support. Additionally use this with four motor six wheel drives.
     */
    FOURWD,

    /**
     * For use with six motor, six wheel drivetrains. For six wheel four wheel motor drivetrains use FOURWD!
     */
    SIXWD,
    /**
     * Differential swerve type drivetrain. Assumes two driving wheels centered along the middle of the robot.
     */
    DIFFY

}
