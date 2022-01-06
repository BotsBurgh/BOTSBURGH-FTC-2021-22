package org.firstinspires.ftc.teamcode.api.bp;

import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor.Color;

public interface SensorRobot {
    default void driveToColor(SmartColorSensor sensor, Color color, double power) {}
    default void driveToColor(SmartColorSensor sensor, Color color) {
        // this.driveToColor(sensor, color, DEFAULT_POWER);
    }

    default void driveToWhiteLine(SmartColorSensor sensor) {
        this.driveToColor(sensor, Color.WHITE);
        // Or this.driveToWhiteLine(sensor, DEFAULT_POWER);
    }
    default void driveToWhiteLine(SmartColorSensor sensor, double power) {
        this.driveToColor(sensor, Color.WHITE, power);
    }
}
