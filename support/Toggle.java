package org.firstinspires.ftc.teamcode.auto.support;

/**
 * Toggle Object - code to detect button presses and update a toggle variable in a fast loop
 * **/
public class Toggle {
    /**
     * The state (true or false) that this object is set to
     */
    private boolean toggleState;
    /**
     * The past input that was provided to this object
     */
    private boolean pastInputState;

    /**
     * Writes the toggle state to a variable
     * Additionally the past state is as a default set to startingToggleState.
     * @param startingToggleState the state the toggle should initially start in
     */
    public Toggle(boolean startingToggleState) {
        toggleState = startingToggleState;
        pastInputState = startingToggleState;
    }

    /**
     * Get the value of the toggle
     * @return the value of the toggle
     * Precondition:  the object's starting conditions have been assigned correctly
     * Postcondition: the appropriate state is returned
     */
    public final boolean getToggleState(){
        return toggleState;
    }

    /**
     * updates the current state according to a leading edge detector
     * @param currentInputState what the input currently reads (ex. a button on the control pad)
     * Precondition:  currentInputState accurately reflects the toggle state
     * Postcondition: the object's state has been updated according to a leading edge detector
     */
    public final void updateLeadingEdge(boolean currentInputState) {
        // if the past input state was false and the current input state is true then
        // otherwise the current toggle state is false
        if (!pastInputState && currentInputState)
            // if the past toggle state was false then the current toggle state is true
            toggleState = !toggleState;

        // set the past state to the last read state
        pastInputState = currentInputState;
    }

    /**
     * updates the current state according to a falling edge detector
     * @param currentInputState what the input currently reads (ex. a button on the control pad)
     * Precondition:  currentInputState accurately reflects the toggle state
     * Postcondition: the object's state has been updated according to a falling edge detector
     */
    public final void updateFallingEdge(boolean currentInputState) {
        // if the past input state was false and the current input state is true then
        // otherwise the current toggle state is false
        if (pastInputState && !currentInputState)
            // if the past toggle state was false then the current toggle state is true
            toggleState = !toggleState;

        // set the past state to the last read state
        pastInputState = currentInputState;
    }
}
