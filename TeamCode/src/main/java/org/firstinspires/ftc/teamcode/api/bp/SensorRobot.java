package org.firstinspires.ftc.teamcode.api.bp;

import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor.Color;

public interface SensorRobot {
    // Allow having a power default?
    default void driveToColor(SmartColorSensor sensor, Color color, double power) {}
    default void driveToWhiteLine(SmartColorSensor sensor, double power) {
        this.driveToColor(sensor, Color.WHITE, power);
    }
}
