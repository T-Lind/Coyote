package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import java.util.ArrayList;

public abstract class PathSequenceFather {
    // Common variables and objects among all path sequences
    /**
     * List of the paths to follow
     */
    protected ArrayList<Path> trajectory;

    /**
     * Radius of the wheel
     */
    protected double wheelRadius;


    /**
     * Build each Path in the trajectory.
     * Precondition:  trajectory is not null
     * Postcondition: all paths have been successfully built
     */
    protected final void buildAll(){
        if(trajectory == null)
            throw new RuntimeException("Trajectory in PathSequenceFather.buildAll() must not be null!");
        for(Path path : trajectory)
            path.build();
    }
    /**
     * Build a specific trajectory
     * @param i the index of the Path that you want to build.
     * Precondition:  i is a valid index and trajectory is not null
     * Postcondition: the Path at position i has been built.
     */
    protected final void build(int i){
        if(trajectory == null)
            throw new RuntimeException("Trajectory in PathSequenceFather.build(...) must not be null!");
        if(i < 0)
            throw new RuntimeException("Index in PathSequenceFather.buildAll() is not valid!");
        trajectory.get(i).build();
    }

    /**
     * Precondition:  all paths have been build using the buildAll() method.
     * Note that this is not technically necessary but reduces lag time.
     * Sets the completed state of each path back to false to allow for path reusability
     * Postcondition: only the completed variable in each Path in trajectory has been changed
     */
    protected final void resetPaths(){
        if(trajectory == null)
            throw new RuntimeException("Trajectory in PathSequenceFather.resetPaths() must not be null!");

        for(Path path : trajectory)
            path.setCompleted(false);
    }

    /**
     * Method to follow the path sequence
     */
    public abstract void follow();
}
