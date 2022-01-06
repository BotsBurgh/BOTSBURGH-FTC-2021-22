package org.firstinspires.ftc.teamcode.api.hw;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Button {
    private final DigitalChannel button;

    /**
     * Button constructor
     * @param button Digital Channel for the button
     */
    Button(DigitalChannel button) {
        this.button = button;
    }

    /**
     * Checks if the button is pressed
     * @return Status of button. True is pressed, false is not pressed
     */
    public boolean pressed() {
        return !button.getState();
    }
}
