package org.firstinspires.ftc.teamcode.api;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.api.bp.AbstractRobot;
import org.firstinspires.ftc.teamcode.api.vars.Hardware;

public class Robot extends AbstractRobot {
    public DcMotor frontLeft, frontRight, backLeft, backRight, duck;

    public Robot() {
        super();
    }

    public Robot(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void initTeleOp(@NonNull OpMode opMode) {
        this.frontLeft = opMode.hardwareMap.get(DcMotor.class, Hardware.FRONT_LEFT);
        this.frontRight = opMode.hardwareMap.get(DcMotor.class, Hardware.FRONT_RIGHT);
        this.backLeft = opMode.hardwareMap.get(DcMotor.class, Hardware.BACK_LEFT);
        this.backRight = opMode.hardwareMap.get(DcMotor.class, Hardware.BACK_RIGHT);
        this.duck = opMode.hardwareMap.get(DcMotor.class, Hardware.DUCK);
    }

    @Override
    public void moveAll(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }

    @Override
    public void stopAll() {
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
        this.backLeft.setPower(0);
        this.backRight.setPower(0);
        this.duck.setPower(0);
    }

    @Override
    public void moveFront(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
    }

    @Override
    public void moveBack(double power) {
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }

    public void spinDuck(double power) {
        this.duck.setPower(power);
    }

    public void setMotorsDirection(@NonNull DcMotor[] motors, DcMotor.Direction direction) {
        for (DcMotor motor: motors) {
            motor.setDirection(direction);
        }
    }

    public void reverseMotors(@NonNull DcMotor[] motors) {
        this.setMotorsDirection(motors, DcMotor.Direction.REVERSE);
    }
}