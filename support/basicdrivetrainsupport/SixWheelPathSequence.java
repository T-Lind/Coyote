package org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.KalmanFilter;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Path;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PIDController;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequenceFather;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PeripheralType;

import java.util.ArrayList;

/**
 * Program to take linear velocities from each wheel and translate
 * them into 6wd
 * Created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */
public class SixWheelPathSequence extends PathSequenceFather {
    /**
     * First left motor object (order doesn't matter but for the sake of argument let's start with the rearmost motors on the robot)
     */
    private DcMotorEx left1;
    /**
     * Second left motor object
     */
    private DcMotorEx left2;
    /**
     * Third left motor object
     */
    private DcMotorEx left3;
    /**
     * First right motor object (rearmost right motor)
     */
    private DcMotorEx right1;
    /**
     * Second right motor object
     */
    private DcMotorEx right2;
    /**
     * Third right motor object
     */
    private DcMotorEx right3;

    /**
     * Constructor that assigns the objects used in SixWheelPathSequence
     * @param paths is the ArrayList of paths
     * @param left1 and is a left motor (presumed to be negative to go forward) does not matter which
     * @param left2 and is a left motor (presumed to be negative to go forward) does not matter which
     * @param left3 and is a left motor (presumed to be negative to go forward) does not matter which
     * @param right1 is the right motor (presumed to be positive to go forward) does not matter which
     * @param right2 is the right motor (presumed to be positive to go forward) does not matter which
     * @param right3 is the right motor (presumed to be positive to go forward) does not matter which
     * @param wheelR is the wheel's radius
     */
    public SixWheelPathSequence(ArrayList<Path> paths, DcMotorEx left1, DcMotorEx left2, DcMotorEx left3, DcMotorEx right1, DcMotorEx right2, DcMotorEx right3, double wheelR){
        trajectory = paths;
        wheelRadius = wheelR;

        this.left1 = left1;
        this.left2 = left2;
        this.left3 = left3;
        this.right1 = right1;
        this.right2= right2;
        this.right3= right3;

    }

    /**
     * Actually moves the robot along the specified Paths.
     * Precondition:  the motor and trajectory objects have been created
     * Postcondition: the path has been followed
     */
    @Override
    public final void follow(){
        if(left1 == null || left2 == null || left3 == null || right1 == null || right2 == null || right3 == null || trajectory == null)
            throw new RuntimeException("Null object parameter passed to SixWheelPathSequence (in SixWheelPathSequence.follow())");

        ElapsedTime t = new ElapsedTime();
        t.reset();

        for(Path p : trajectory){
            if(!p.getBuilt())
                p.build();

            KalmanFilter kLeft1 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidLeft1 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

            KalmanFilter kLeft2 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidLeft2 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

            KalmanFilter kLeft3 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidLeft3 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

            KalmanFilter kRight1 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidRight1 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

            KalmanFilter kRight2 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidRight2 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

            KalmanFilter kRight3 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidRight3 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

            double offset = t.milliseconds();

            // Execute the path
            while(!p.getCompleted()){
                // Get the velocities from what the path says the end result velocities should be
                double leftV = Path.convertForStandardDrivetrain(wheelRadius, p.getLeftVelocity((t.milliseconds()-offset)/1000));
                double rightV = Path.convertForStandardDrivetrain(wheelRadius, p.getRightVelocity((t.milliseconds()-offset)/1000));

                // Correct based on PID and kalman filter
                double corL1 = pidLeft1.update((long)leftV, (long)kLeft1.filter(left1.getVelocity(RADIANS)));
                double corL2 = pidLeft2.update((long)leftV, (long)kLeft2.filter(left2.getVelocity(RADIANS)));
                double corL3 = pidLeft3.update((long)leftV, (long)kLeft3.filter(left3.getVelocity(RADIANS)));
                double corR1 = pidRight1.update((long)rightV, (long)kRight1.filter(right1.getVelocity(RADIANS)));
                double corR2 = pidRight2.update((long)rightV, (long)kRight2.filter(right2.getVelocity(RADIANS)));
                double corR3 = pidRight3.update((long)rightV, (long)kRight3.filter(right3.getVelocity(RADIANS)));

                // Write the corrected values
                left1.setVelocity(corL1+leftV, RADIANS);
                left2.setVelocity(corL2+leftV, RADIANS);
                left3.setVelocity(corL3+leftV, RADIANS);

                right1.setVelocity(corR1+rightV, RADIANS);
                right2.setVelocity(corR2+rightV, RADIANS);
                right3.setVelocity(corR3+rightV, RADIANS);
            }
            resetPaths();
        }
    }
}
