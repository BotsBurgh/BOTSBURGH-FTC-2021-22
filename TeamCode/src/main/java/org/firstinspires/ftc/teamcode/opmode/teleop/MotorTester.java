package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.vars.Group;

@TeleOp(name = "Motor Tester", group= Group.UTILITIES)
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                telemetry.addData(">", "Front Left");
                robot.frontLeft.setPower(1);
            } else {
                robot.frontLeft.setPower(0);
            }

            if (gamepad1.b) {
                telemetry.addData(">", "Front Right");
                robot.frontRight.setPower(1);
            } else {
                robot.frontRight.setPower(0);
            }

            if (gamepad1.x) {
                telemetry.addData(">", "Back Left");
                robot.backLeft.setPower(1);
            } else {
                robot.backLeft.setPower(0);
            }

            if (gamepad1.y) {
                telemetry.addData(">", "Back Right");
                robot.backRight.setPower(1);
            } else {
                robot.backRight.setPower(0);
            }
        }
    }
}