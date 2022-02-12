package org.firstinspires.ftc.teamcode.api.bp;

@Deprecated
public interface StepWheeledRobot {
    default void powerStepWheels(double leftPower, double rightPower) {}
    default void powerStepWheels(double power) {this.powerStepWheels(power, power);};
}
