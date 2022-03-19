package org.firstinspires.ftc.teamcode.api.bp;

public interface ArmRobot {
    default void positionArm(double position) {
    }

    default void adjustArm(double amount) {
    }

    default void positionClaw(double position) {
    }

    default void adjustClaw(double amount) {
    }

    default void openClaw() {
    }

    default void closeClaw() {
    }
}
