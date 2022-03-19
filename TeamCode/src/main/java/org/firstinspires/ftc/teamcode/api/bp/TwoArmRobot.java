package org.firstinspires.ftc.teamcode.api.bp;

public interface TwoArmRobot extends ArmRobot {
    default void positionArm2(double position) {
    }

    default void adjustArm2(double amount) {
    }

    default void positionClaw2(double position) {
    }

    default void adjustClaw2(double amount) {
    }

    default void openClaw2() {
    }

    default void closeClaw2() {
    }
}
