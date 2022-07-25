package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

public class RotatePods extends Path{
    /**
     * Method returning the type of path this specific path is
     *
     * @return the correct type of path.
     *
     */
    @Override
    public PathType getType() {
        return PathType.PODTURN;
    }

    /**
     * Meant to compute the trajectory and essentially turn it into a piecewise function.
     * This method is supposed to be overriden.
     * <p>
     * Precondition:  the construction variable has been instantiated
     * Postcondition: the path is set to a built status
     */
    @Override
    public void build() {

    }
}
