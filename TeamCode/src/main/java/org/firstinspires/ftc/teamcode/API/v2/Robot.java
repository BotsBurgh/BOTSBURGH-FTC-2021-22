package org.firstinspires.ftc.teamcode.API.v2;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.API.v2.bp.AbstractRobot;

public class Robot extends AbstractRobot {
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void initTeleOp(@NonNull OpMode opMode) {
        this.frontLeft = opMode.hardwareMap.get(DcMotor.class, Naming.FRONT_LEFT);
        this.frontRight = opMode.hardwareMap.get(DcMotor.class, Naming.FRONT_RIGHT);
        this.backLeft = opMode.hardwareMap.get(DcMotor.class, Naming.BACK_LEFT);
        this.backRight = opMode.hardwareMap.get(DcMotor.class, Naming.BACK_RIGHT);
    }

    @Override
    public void moveAll(double power) {
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
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
}
