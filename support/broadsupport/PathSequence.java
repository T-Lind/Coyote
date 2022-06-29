package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport.FourWheelPathSequence;
import org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport.SixWheelPathSequence;
import org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport.TwoWheelPathSequence;
import org.firstinspires.ftc.teamcode.auto.support.diffysupport.DiffyPathSequence;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.Drivetrain;

import java.util.ArrayList;

/**
 * Wrapper for all the different types of Path Sequences, works closely with PathSequenceFather
 * where the path sequences inherit common data.
 *
 */
public class PathSequence {
    // Items necessary for each path - trajectory to follow and the time of drivetrain
    /**
     * Object of sequence to follow
     */
    private PathSequenceFather sequence;
    /**
     * Type of drivetrain to run the sequence on
     */
    private Drivetrain drivetrainType;

    /**
     * Path sequence constructor for two wheel drives
     * @param drivetrainType the type of drivetrain used (TWOWD, FOURWD, SIXWD, DIFFY)
     * @param paths the list of paths for the robot to follow
     * @param left the left motor object
     * @param right the right motor object
     * @param wheelR the radius of the wheel
     */
    public PathSequence(Drivetrain drivetrainType, ArrayList<Path> paths, DcMotorEx left, DcMotorEx right, double wheelR){
        sequence = new TwoWheelPathSequence(paths, left, right, wheelR);
        this.drivetrainType = drivetrainType;
    }

    /**
     * Constructor for both FOURWD and DIFFY type drivetrains. IF statement needed to distinguish
     * based on drivetrainType
     * @param drivetrainType the type of drivetrain used (TWOWD, FOURWD, SIXWD, DIFFY)
     * @param paths the list of paths for the robot to follow
     * @param left1 the first left motor object (order does not matter)
     * @param left2 the second left motor object (order does not matter)
     * @param right1 the first right motor object (order does not matter)
     * @param right2 the second right motor object (order does not matter)
     * @param wheelR the radius of the wheel
     */
    public PathSequence(Drivetrain drivetrainType, ArrayList<Path> paths, DcMotorEx left1, DcMotorEx left2, DcMotorEx right1, DcMotorEx right2, double wheelR){
        if( drivetrainType == Drivetrain.FOURWD)
            sequence = new FourWheelPathSequence(paths, left1, left2, right1, right2, wheelR);
        else if( drivetrainType == Drivetrain.DIFFY)
            sequence = new DiffyPathSequence(paths, left1, left2, right1, right2, wheelR);
        this.drivetrainType = drivetrainType;
    }

    /**
     * Constructor for the SIXWD type drivetrain
     * @param drivetrainType the type of drivetrain used (TWOWD, FOURWD, SIXWD, DIFFY)
     * @param paths the list of paths for the robot to follow
     * @param left1 the first left motor object (order does not matter)
     * @param left2 the second left motor object (order does not matter)
     * @param left3 the third left motor object (order does not matter)
     * @param right1 the first right motor object (order does not matter)
     * @param right2 the second right motor object (order does not matter)
     * @param right3 the third right motor object (order does not matter)
     * @param wheelR the radius of the wheel
     */
    public PathSequence(Drivetrain drivetrainType, ArrayList<Path> paths, DcMotorEx left1, DcMotorEx left2, DcMotorEx left3, DcMotorEx right1, DcMotorEx right2, DcMotorEx right3, double wheelR){
        sequence = new SixWheelPathSequence(paths, left1, left2, left3, right1, right2, right3, wheelR);
        this.drivetrainType = drivetrainType;
    }

    /**
     * Method to get the path sequence
     * @return the path sequence
     * Precondition:  sequence has been created and is not null
     * Postcondition: the path sequence is returned successfully
     */
    public final PathSequenceFather getPathSequence(){
        if(sequence == null)
            throw new RuntimeException("Cannot run PathSequence.getPathSequence() if the sequence is null!");
        return sequence;
    }

    /**
     * Build all of the paths in the passed sequence object
     * Precondition:  sequence is not null and has been instantiated
     * Postcondition: each path in sequence has been built
     */
    public final void buildAll(){
        if(sequence == null)
            throw new RuntimeException("Cannot run PathSequence.buildAll() if the sequence is null!");
        sequence.buildAll();
    }

    /**
     * Actually follow the passed sequence object
     * Precondition:  sequence is not null and has been instantiated
     * Postcondition: every sequence has been followed by the robot
     */
    public final void follow(){
        if(sequence == null)
            throw new RuntimeException("Cannot run PathSequence.follow() if the sequence is null!");
        sequence.follow();
    }
}
