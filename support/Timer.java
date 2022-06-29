package org.firstinspires.ftc.teamcode.auto.support;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Basic class to make a simple time keeping object
 */
public class Timer {
    /**
     * The time the timer should loop for before returning a true value
     */
    private double timeToLoop;

    /**
     * Time keeping object
     */
    private ElapsedTime elapsedTime;

    /**
     * Acceptable margin of error for time measurement in seconds for the timer
     */
    private static final double TIMER_MARGIN_OF_ERROR = 0.01;

    /**
     * Establish a timer object. Not used for the static method sleep.
     * @param timeToLoop the amount of time to be in a loop.
     */
    public Timer(double timeToLoop){
        this.timeToLoop = timeToLoop;
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    /**
     * Check to see if the current time for this Timer instance is past the time to loop
     * @return whether or not to continue looping
     * Precondition:  elapsedTime has been assigned
     * Postcondition: returns the correct state of the timer
     */
    public final boolean inTimeRange(){
        if(elapsedTime == null)
            throw new RuntimeException("elaspsedTime in Timer.inTimeRange() cannot be null!");

        return elapsedTime.milliseconds()/1000.0+TIMER_MARGIN_OF_ERROR < timeToLoop;
    }

    /**
     * Gets the current time of this Timer instance
     * @return the current time in seconds of this Timer instance
     * Precondition:  elapsedTime has been assigned
     * Postcondition: returns the correct time of the timer
     */
    public final double getTime(){
        if(elapsedTime == null)
            throw new RuntimeException("elaspsedTime in Timer.getTime() cannot be null!");

        return elapsedTime.milliseconds()/1000;
    }

    /**
     * Hang up the thread based on a time to sleep
     * @param timeToSleep the time the thread should sleep for
     * Postcondition: the thread has slept for the appropriate time
     */
    public static void sleep(double timeToSleep){
        ElapsedTime currentTime = new ElapsedTime();

        double marginedCurrentTime;
        do{
            marginedCurrentTime = currentTime.milliseconds()/1000+ TIMER_MARGIN_OF_ERROR;
        }
        while(marginedCurrentTime < timeToSleep);
    }

}
