package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

/**
 * A class for running the markers on different threads at the same time to run each marker.
 * You can create as many threads as you want!
 *
 */
public class RunnableCollective implements Runnable{
    /**
     * Create a MarkerList - essentially an object containing the Interface InsertMarkers
     * which are realized with a lambda expression
     * (which are the things we want to do like raise an arm) and the times to execute them.
     */
    private final MarkerList markerList;

    /**
     * Signals the program to shut off the markers
     */
    private boolean stopMarkers;

    /**
     * Thread running all the other markers
     */
    private Thread mainThread;

    /**
     * Static array containing all the other threads running the other markers
     */
    private Thread[] childThreadList;

    /**
     * State array preventing repeated running of threads
     */
    private boolean[] threadStates;

    /**
     * Instantiate the object with a markerList to reference, used in the PathSequence family
     * @param markerList is the collection of markers and the time they should execute at
     */
    public RunnableCollective(MarkerList markerList){
        if(markerList == null)
            throw new RuntimeException("markerList in RunnableCollectiveArray.RunnableCollectiveArray(...) must not be null!");

        this.markerList = markerList;
        childThreadList = new Thread[markerList.getMarkers().length];
        stopMarkers = false;

        threadStates = new boolean[markerList.getMarkers().length];
        Arrays.fill(threadStates, false);
    }

    /**
     * Start the thread that runs all of the markers
     * Precondition:  markerList is not null and has been instantiated
     * Postcondition: the main thread has started
     */
    public final void activateMarkers(){
        if(markerList == null)
            throw new RuntimeException("markerList in RunnableCollectiveArray.activateMarkers() must not be null!");

        mainThread = new Thread(this);
        mainThread.start();
    }


    /**
     * Set the stop condition and interrupts threads
     * Precondition:  childThreadList has been instantiated and is not null
     * Postcondition: the markers have stopped running
     */
    public final void setStopMarkers() {
        if(markerList == null)
            throw new RuntimeException("markerList in RunnableCollectiveArray.setStopMarkers() must not be null!");
        if(childThreadList == null)
            throw new RuntimeException("childThreadList in RunnableCollectiveArray.setStopMarkers() must not be null!");

        // Interrupt any child thread currently being used
        stopMarkers = true;
        for (Thread thread : childThreadList)
            if (thread != null && thread.isAlive()) {
                thread.interrupt();
            }

        // Interrupt the parent thread
        mainThread.interrupt();
    }

    /**
     * The broad run method - since the RunnableCollective itself is in a thread it needs to have
     * a run method, which is started whenever the thread is. It instantiates the new threads
     * and starts them as the time becomes right.
     * Precondition:  childThreadList and markerList has been instantiated and are not null
     * Postcondition: each child thread has been started and executed according to the interface
     * realization with lambda in the runner program
     */
    @Override
    public final void run(){
        if(markerList == null)
            throw new RuntimeException("markerList in RunnableCollectiveArray.run() must not be null!");
        if(childThreadList == null)
            throw new RuntimeException("childThreadList in RunnableCollectiveArray.setStopMarkers() must not be null!");

//
        // Declare the ElapsedTime object here to reduce lag after runMarkers is true
        ElapsedTime markerTime = new ElapsedTime();

        // Reset the time the markers run off of (should be whenever start button is pressed)
        markerTime.reset();

        // Create each new thread in the array
        for(int i=0;i< childThreadList.length;i++)
            childThreadList[i] = new Thread(this.new ChildThread(i));

        // Look and start the threads if the time is right and they have not been started before
        while(!stopMarkers)
            for(int i=0;i<childThreadList.length;i++){
                if(markerList.getTime(i) < markerTime.milliseconds()/1000 && !threadStates[i] && !childThreadList[i].isAlive()){
                    childThreadList[i].start();
                    setThreadComplete(i);
                }
            }
    }

    /**
     * Set a thread to a complete status - don't run it multiple times
     * @param i the index at which to set a thread to a complete status
     */
    private void setThreadComplete(int i){
        if(i < 0 || i >= markerList.getMarkers().length)
            throw new RuntimeException("i must be a valid index in RunnableCollective.setThreadComplete(...)!");

        threadStates[i] = true;
        childThreadList[i].interrupt();
    }

    /**
     * Private thread object - correlates to a specific index of the markerList.
     */
    private final class ChildThread implements Runnable{
        /**
         * Which thread to run
         */
        private final int index;


        /**
         * Assign an index to this thread
         * @param index the index of which thread to execute
         */
        public ChildThread(int index){
            this.index = index;
        }

        /**
         * Method to execute the InsertMarker
         *Precondition:  markerList is not null, and index has been assigned
         *Postcondition: the marker at this spot has started
         */
        @Override
        public void run() {
            markerList.getInsertMarker(index).execute();
        }
    }
}
