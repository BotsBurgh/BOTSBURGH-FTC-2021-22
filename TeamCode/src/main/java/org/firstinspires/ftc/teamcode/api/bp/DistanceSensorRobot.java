package org.firstinspires.ftc.teamcode.api.bp;

public interface DistanceSensorRobot {
    default public double getFrontDistance() {
        return (
            this.getFLDistance() + this.getFRDistance()
        ) / 2;
    }

    default public double getBackDistance() {
        return (
            this.getBLDistance() + this.getBRDistance()
        ) / 2;
    }

    public double getFLDistance();
    public double getFRDistance();
    public double getBLDistance();
    public double getBRDistance();
}
