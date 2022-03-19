package org.firstinspires.ftc.teamcode.api.bp;

public interface DistanceSensorRobot {
    default double getFrontDistance() {
        return (
            this.getFLDistance() + this.getFRDistance()
        ) / 2;
    }

    default double getBackDistance() {
        return (
            this.getBLDistance() + this.getBRDistance()
        ) / 2;
    }

    double getFLDistance();
    double getFRDistance();
    double getBLDistance();
    double getBRDistance();
}
