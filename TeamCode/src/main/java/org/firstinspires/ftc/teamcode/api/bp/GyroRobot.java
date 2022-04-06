package org.firstinspires.ftc.teamcode.api.bp;

public interface GyroRobot {
    default void gyroDrive(double speed, double distance, double power) {
    }

    default void gyroHold(double speed, double angle, double holdTime) {
    }

    default boolean onHeading(double speed, double angle, double PCoeff) {
        return false;
    }

    default double getError(double targetAngle) {
        return targetAngle;
    }
}
