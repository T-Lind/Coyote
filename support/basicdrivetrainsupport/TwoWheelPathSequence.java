package org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.KalmanFilter;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Path;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PIDController;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequenceFather;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

/**
 * Program to take linear velocities from each wheel and translate
 * them into 2wd
 * Created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */
public class TwoWheelPathSequence extends PathSequenceFather{
    /**
     * Left motor object on your drivetrain
     */
    private DcMotorEx left;

    /**
     * Right motor object on your drivetrain
     */
    private DcMotorEx right;
    /**
     * Constructor for TwoWheelPathSequence to assign used objects
     * @param paths is the ArrayList of paths
     * @param left is the left motor (presumed to be negative to go forward)
     * @param right is the right motor (presumed to be positive to go forward)
     * @param wheelR is the wheel's radius
     */
    public TwoWheelPathSequence(ArrayList<Path> paths, DcMotorEx left, DcMotorEx right, double wheelR){
        trajectory = paths;
        wheelRadius = wheelR;

        this.left = left;
        this.right = right;

    }


    /**
     * Actually moves the robot along the specified Paths.
     * Precondition:  the motor and trajectory objects have been created
     * Postcondition: the path has been followed
     */
    public final void follow(){
        if(left == null || right == null || trajectory == null)
            throw new RuntimeException("Null object parameter passed to TwoWheelPathSequence (in TwoWheelPathSequence.follow())");

        ElapsedTime t = new ElapsedTime();
        t.reset();

        for(Path p : trajectory){
            if(!p.getBuilt())
                p.build();

            KalmanFilter k3;
            PIDController pid3;
            KalmanFilter k4;
            PIDController pid4;

            // Experimental bit here
            if(p.getType() == PathType.LINE || p.getType() == PathType.SPLINE){

                k3 = new KalmanFilter();
                pid3 = new PIDController();

                k4 = new KalmanFilter();
                pid4 = new PIDController();
            }
            else if(p.getType() == PathType.TURN){
                k3 = new KalmanFilter();
                pid3 = new PIDController();

                k4 = new KalmanFilter();
                pid4 = new PIDController();
            }
            else{
                k3 = new KalmanFilter();
                pid3 = new PIDController();

                k4 = new KalmanFilter();
                pid4 = new PIDController();
            }

            double offset = t.milliseconds();
            while(!p.getCompleted()){
                double leftV = Path.convertForStandardDrivetrain(wheelRadius, p.getLeftVelocity((t.milliseconds()-offset)/1000));
                double rightV = Path.convertForStandardDrivetrain(wheelRadius, p.getRightVelocity((t.milliseconds()-offset)/1000));

                double corL = pid3.update((long)leftV, (long)k3.filter(left.getVelocity(RADIANS)));
                double corR = pid4.update((long)rightV, (long)k4.filter(right.getVelocity(RADIANS)));

                left.setVelocity(corL+leftV, RADIANS);
                right.setVelocity(corR+rightV, RADIANS);
            }

            resetPaths();
        }
    }

}
