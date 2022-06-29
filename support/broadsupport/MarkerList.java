package org.firstinspires.ftc.teamcode.auto.support.broadsupport;
import java.util.Arrays;

/**
 * Supporting file to create a static array of markers.
 * @author Tiernan Lindauer
 * For team 7797 Victorian Voltage.
 */
public class MarkerList {
    /**
     * The list of temporal insert markers
     */
    private InsertMarker[] markers;
    /**
     * The list of times at which to execute those markers
     */
    private double[] times;

    /**
     * All of these methods are just meant to nicely create a static array of the InsertMarker implementation.
     * If more than four markers are passed then you must first create them as separate static arrays
     * @param m the marker (listed m, m2, m3, etc.)
     * @param time1 the corresponding time to the first marker
     */
    public MarkerList(InsertMarker m, double time1){
        markers = new InsertMarker[1];
        markers[0] = m;
        times = new double[1];
        setTimesMax();
        times[0] = time1;
    }
    public MarkerList(InsertMarker m, double time1, InsertMarker m2, double time2){
        markers = new InsertMarker[2];
        markers[0] = m;
        markers[1] = m2;
        times = new double[2];
        setTimesMax();
        times[0] = time1;
        times[1] = time2;
    }
    public MarkerList(InsertMarker m, double time1, InsertMarker m2, double time2,
                      InsertMarker m3, double time3){
        markers = new InsertMarker[3];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        times = new double[3];
        setTimesMax();

        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
    }
    public MarkerList(InsertMarker m, double time1, InsertMarker m2, double time2,
                      InsertMarker m3, double time3, InsertMarker m4, double time4){
        markers = new InsertMarker[4];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        times = new double[4];
        setTimesMax();

        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
    }

    /**
     * Create a list of more than four markers
     * @param markers the list of markers to create
     * @param times the corresponding times to execute those markers
     * Precondition:  markers and times must be of the same length
     */
    public MarkerList(InsertMarker[] markers, double[] times){
        if(markers.length != times.length)
            throw new RuntimeException("The length of markers and times must be the same in MarkerList.MarkerList(...)!");
        this.markers = markers;
        this.times = times;
    }
    /**
     * Fill the times to the maximum value possible, so that unused InsertMarker slots do not create new threads
     * Precondition:  markers is not null
     * Postcondition: every item slot in the array has the maximum value possible so as to not create
     * empty threads, taking up memory.
     */
    private void setTimesMax(){
        if(markers == null)
            throw new RuntimeException("Cannot run MarkerList.setTimesMax() while markers is null!");
        Arrays.fill(times, Double.MAX_VALUE);
    }

    /**
     * Get the full list of markers
     * @return all of the markers
     * Precondition:  markers has been set
     * Postcondition: the list of markers is returned successfully.
     */
    public final InsertMarker[] getMarkers(){
        if(markers == null)
            throw new RuntimeException("Cannot run MarkerList.getMarkers() while markers is null!");
        return markers;
    }

    /**
     * Get the InsertMarker at position i
     * @param i the index to get the InsertMarker in the markers list
     * @return The InsertMarker alias at i
     * Precondition:  i >= 0
     * Postcondition: the InsertMarker at index i is returned successfully
     */
    public final InsertMarker getInsertMarker(int i){
        if(i < 0)
            throw new RuntimeException("The index for MarkerList.getInsertMarker(int i) must be a valid index!");
        if(markers == null)
            throw new RuntimeException("Cannot run MarkerList.getInsertMarker() while markers is null!");
        return markers[i];
    }

    /**
     * Get the starting time for the marker at position i
     * @param i the index to get the time in the times list
     * @return the starting time for marker i
     * Precondition:  times has been set
     * Postcondition: the time at which to execute InsertMarker i has been delivered successfully.
     */
    public final double getTime(int i){
        if(i < 0)
            throw new RuntimeException("The index for MarkerList.getTime(int i) must be a valid index!");
        if(times == null)
            throw new RuntimeException("The times array in MarkerList.getTime(int i) must be not null!");
        return times[i];
    }

}
