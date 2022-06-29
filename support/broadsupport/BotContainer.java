package org.firstinspires.ftc.teamcode.auto.support.broadsupport;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraPipelines.TSEDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


/**
 * Class to slim down autonomous programs for four motor drivetrains and execute markers.
 */
abstract public class BotContainer extends LinearOpMode{
   // Pathing and marker list objects
   /**
    * The sequence of paths to follow
    */
   private PathSequence pathSequence;
   /**
    * The markers to follow
    */
   protected MarkerList markerList;
   /**
    * Object to create many threads to run mechanism loops
    */
   private RunnableCollective runMarkerObject;

   /**
    * Name object for the webcam
    */
   protected WebcamName webcameraName;

   /**
    * The actual camera object to apply a pipeline to
    */
   protected OpenCvCamera camera;

   /**
    * A list of paths to reduce user code
    */
   private ArrayList<Path> listOfPaths = new ArrayList<>();

   /**
    * Assign the path sequence to this LinearOpMode
    * @param pathSequence is the set of basic robot instructions to move along
    *                     (final linear wheel velocities).
    * Precondition:  the parameter pathSequence is not null
    * Postcondition: pathSequence is not null
    */
   protected final void setPathSequence(PathSequence pathSequence) {
      if(pathSequence == null)
         throw new RuntimeException("pathSequence parameter in BotContainer.setPathSequence(...) cannot be null!");

      this.pathSequence = pathSequence;
   }

   /**
    * Method to get the private pathSequence object.
    * @return the pathSequence object.
    */
   protected final PathSequence getPathSequence(){
      return pathSequence;
   }

   protected final void addPath(Path path){
      listOfPaths.add(path);
   }

   /**
    * Get the list of paths
    * @return the ArrayList of paths
    */
   protected final ArrayList<Path> getListOfPaths(){
      return listOfPaths;
   }

   /**
    * Follows the path and marker list given, assuming one was given.
    * Precondition:  either the marker list or the pathSequence has been assigned
    * Postcondition: the path has been followed and the markers have stopped
    */
   protected final void executeAuto() {
      if(markerList == null && pathSequence == null)
         throw new RuntimeException("Tried to run an empty path and set of markers in BotContainer.executeAuto(). You must assign at least one!");

      // Only execute the auto when the button is pressed
      waitForStart();

      // Start markers
      if (markerList != null)
         activateMarkers();

      // Follow the path
      if (pathSequence != null)
         pathSequence.follow();

      // Stop the insertMarkers
      if (markerList != null)
         stopMarkers();
   }

   /**
    * Start the markers according to the times given, also check for null
    * Precondition:  markerList has been assigned and is not null
    * Postcondition: the multi-threaded execution of markers has started
    */
   private void activateMarkers(){
      if(markerList == null)
         throw new RuntimeException("Tried to execute markerList in BotContainer.activateMarkers() but markerList was null!");

      runMarkerObject = new RunnableCollective(this.markerList);

      // Start the markers
      runMarkerObject.activateMarkers();
   }

   /**
    * Interrupts all threads and stops the markers
    * Precondition:  markerList has been assigned and is not null
    * Postcondition: the markers has been stopped
    */
   private void stopMarkers(){
      if(markerList == null)
         throw new RuntimeException("Tried to stop markerList in BotContainer.stopMarkers() but markerList was null!");
      runMarkerObject.setStopMarkers();
   }
}
