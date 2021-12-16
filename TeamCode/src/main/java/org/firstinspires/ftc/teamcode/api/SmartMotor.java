package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import lombok.Getter;
import lombok.Setter;

public class SmartMotor {
    @Getter private final DcMotor motor;
    @Getter private double power;

    @Getter @Setter private double powerModifier;

    public SmartMotor(DcMotor motor) {
        this(motor, 1);
    }

    public SmartMotor(DcMotor motor, double powerModifier) {
        this.motor = motor;
        this.power = 0;
        this.powerModifier = powerModifier;
    }

    public void setPower(double power) {
        if (Math.abs(power - this.power) > 0) {
            this.power = power;
            motor.setPower(power * powerModifier);
        }
    }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void setTargetPosition(int targetPosition) {
        motor.setTargetPosition(targetPosition);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    public void setMotorType(MotorConfigurationType motorConfigurationType) {
        motor.setMotorType(motorConfigurationType);
    }
}
