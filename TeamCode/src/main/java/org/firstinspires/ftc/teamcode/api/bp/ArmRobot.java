package org.firstinspires.ftc.teamcode.api.bp;

public interface ArmRobot {
    default void powerDuck(double power) {}

    default void positionArm(double position) {}

    // Adjusts position of arm relative to its current position.
    default void adjustArm(double amount) {
        // this.positionArmBase(servo.getPosition() + amount);
    }

    default void positionClaw(double position) {}
    default void adjustClaw(double amount) {}

    default void openClaw() {
        // this.positionClaw(0.9)
    }
    default void closeClaw() {
        // this.positionClaw(0.2)
    }
}
