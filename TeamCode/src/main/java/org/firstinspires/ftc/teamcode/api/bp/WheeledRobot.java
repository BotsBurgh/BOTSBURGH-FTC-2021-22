package org.firstinspires.ftc.teamcode.api.bp;

public interface WheeledRobot {
    default void powerWheels(double flPower, double frPower, double blPower, double brPower) {}
    default void powerWheels(double power) {
        this.powerWheels(power, power, power, power);
    }
}
