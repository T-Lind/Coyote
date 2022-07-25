package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraPipelines.ImagePipeline;
import org.firstinspires.ftc.teamcode.CameraPipelines.TSEDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.Drivetrain;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.DrivetrainSymmetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

abstract public class Robot extends BotContainer{
    // These variables are protected so autonomous programs can use them easier - assign these to what you're using on your drivetrain
    /**
     * These parameters should be replaced with whatever motors you have on your robot!
     */
    protected DcMotorEx leftFront, leftBack, rightFront, rightBack;

    /**
     * pipeline here is of type TSEDetectionPipeline, replace this with whatever pipeline you've made!
     */
    private final ImagePipeline pipeline = new ImagePipeline();

    /**
     * Define whether the drivetrain is symmetrical or not - if both sides of the robot are powered
     * positively and the robot moves forward then the robot is symmetrical, if it turns then it is
     * asymmetrical.
     */
    private final DrivetrainSymmetry symmetryState = DrivetrainSymmetry.SYMMETRICAL;

    /**
     * wheelR is the radius of the wheel and should be replaced with your robots driven wheel radius
     */
    private final double wheelR = 0.03715;

    /**
     * trackWidth is the distance between the wheels of the robot (viewing down the center of the robot), replace with your track width
     */
    private final double trackWidth = 0.295;




    /***************************************/

    // PID Controller Coefficients
    public final static double kP = 0.4;
    public final static double kI = 0.5;
    public final static double kD = 0.3;

    // Kalman Filter Coefficients
    public final static double R = 18;
    public final static double Q = 6;
    public final static double C = 2.7;
    public final static double B = 10;
    public final static double A = 1.5;

     /***************************************/


    /**
     * Run code while the init button has been pressed
     * Precondition: initializeHardware() has been run
     * Postcondition: the initialization phase has been successful.
     */
    protected final void initialize(){
        createPathSequence();

        if(getPathSequence() != null)
            getPathSequence().buildAll();

        do{
            // In initialization - add your code here

            telemetry.addData("The initialization has started successfully.","");
            telemetry.update();
        }
        while (!opModeIsActive());
    }


    /**
     * Create the path sequence object given certain parameters.
     * Postcondition: if there is something in ListOfPaths then it will be built.
     */
    private void createPathSequence(){
        if(getListOfPaths() != null){
            // Create the path sequence
            PathSequence sequenceToFollow = new PathSequence(
                    Drivetrain.DIFFY,                           // REPLACE WITH WHATEVER DRIVETRAIN YOU HAVE
                    getListOfPaths(),                                // **DO NOT REPLACE
                    leftFront, leftBack, rightFront, rightBack, // REPLACE WITH YOUR MOTOR VARIABLES
                    wheelR);                                    // **DO NOT REPLACE

            // Add the path sequence to the robot
            setPathSequence(sequenceToFollow);
        }
    }

    /**
     * Initialize and set the motors' properties
     * Postcondition: motors have been properly assigned
     */
    private void initMotors(){
        // build the motor objects - add your code here
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        // Set the usage of encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Sets zero power behavior to braking
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initialize the camera and set its properties.
     * Postcondition: camera has been properly assigned
     */
    private void initCamera(){
        // Build the camera objects - most likely you won't need to change this, but if you've renamed your webcam then you will!
//        WebcamName name = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        // Create a new camera object in openCV
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcameraName, cameraMonitorViewId);


        camera.setPipeline(pipeline);

        // Start the camera stream or throws an error
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.update();
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                throw new RuntimeException("Error in camera initialization! Error code "+errorCode);
            }
        });
    }

    /**
     * Initialize all and build the path sequence if not null, relay a successful initialization
     * through the telemetry.
     * Should NOT be modified.
     */
    protected final void initializeHardware(){
        // Initialize the drivetrain symmetry
        Path.setSymmetryState(symmetryState);
        Path.setTrackWidth(trackWidth);

        // Initialize motors and camera
        initMotors();
        initCamera();

    }
}
