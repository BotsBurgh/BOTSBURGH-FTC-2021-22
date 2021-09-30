package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.api.Constants;
import org.firstinspires.ftc.teamcode.api.Movement;
import org.firstinspires.ftc.teamcode.api.Robot;

@TeleOp(name = "Forward!", group = "99-test")
public class Forward extends LinearOpMode {
    private static final double MAX_SPEED = 0.8;
    @Override
    public void runOpMode() {
        DcMotorEx bl = this.hardwareMap.get(DcMotorEx.class, Constants.MOTOR_BL);
        DcMotorEx br = this.hardwareMap.get(DcMotorEx.class, Constants.MOTOR_BR);
        DcMotorEx fl = this.hardwareMap.get(DcMotorEx.class, Constants.MOTOR_FL);
        DcMotorEx fr = this.hardwareMap.get(DcMotorEx.class, Constants.MOTOR_FR);

        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Movement.motors.put(Constants.MOTOR_FL, fl);
        Movement.motors.put(Constants.MOTOR_FR, fr);
        Movement.motors.put(Constants.MOTOR_BL, bl);
        Movement.motors.put(Constants.MOTOR_BR, br);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.x) {
                Robot.movement.move(1);
            } else if (gamepad1.y) {
                Robot.movement.move(0.3);
            } else if (gamepad1.a) {
                Robot.movement.move(-0.2);
            } else {
                Robot.movement.move(0);
            }
        }
    }
}
