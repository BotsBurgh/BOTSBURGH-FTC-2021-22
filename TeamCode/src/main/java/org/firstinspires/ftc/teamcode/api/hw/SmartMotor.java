/*
 * Copyright 2020 FIRST Tech Challenge Team 11792
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Inspiration from Out of the Box FTC Team 7244
 * (https://github.com/OutoftheBoxFTC/SkyStone/blob/EeshwarTesting/TeamCode/src/main/java/HardwareSystems/HardwareDevices/SmartMotor.java)
 */

package org.firstinspires.ftc.teamcode.api.hw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import lombok.Getter;
import lombok.Setter;

public class SmartMotor {
    private final DcMotorEx motor;
    @Getter private double power;
    private double oldPower;

    // Motor configuration
    @Getter @Setter double powerModifier = 1;
    @Getter @Setter int ticksPerRev; // please set

    public SmartMotor(DcMotorEx motor, int ticksPerRev) {
        this.motor = motor;
        this.ticksPerRev = ticksPerRev;
        power = 0;
        oldPower = 0;
    }

    public SmartMotor(DcMotorEx motor) {
        this(motor, 2048);
    }

    public void setPower(double power) {
        if (Math.abs(power - oldPower) > 0) {
            oldPower = power;
            this.power = power;
            motor.setPower(power*powerModifier);
        }
    }

    public void setDirection(DcMotorEx.Direction direction) {
        motor.setDirection(direction);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setMode(DcMotorEx.RunMode runMode) {
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

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients compensatedCoefficients) {
        motor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }
}

