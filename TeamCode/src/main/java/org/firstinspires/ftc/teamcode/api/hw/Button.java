package org.firstinspires.ftc.teamcode.api.hw;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Button {
    private final DigitalChannel button;

    Button(DigitalChannel button) {
        this.button = button;
    }

    public boolean pressed() {
        return !button.getState();
    }
}
