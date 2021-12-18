package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.vars.Group;

@TeleOp(name = "Move with Controller", group= Group.UTILITIES)
public class MoveController extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            double flPower = Range.clip(( x1 - y1 + rotation), -1.0, 1.0);
            double blPower = Range.clip((-x1 - y1 + rotation), -1.0, 1.0);
            double brPower = Range.clip(( x1 - y1 - rotation), -1.0, 1.0);
            double frPower = Range.clip((-x1 - y1 - rotation), -1.0, 1.0);

            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);
            robot.backRight.setPower(brPower);
            robot.backLeft.setPower(blPower);

            if (gamepad1.left_bumper) {
                robot.spinDuck(0.7);
            } else if (gamepad1.right_bumper) {
                robot.spinDuck(-0.7);
            } else {
                robot.spinDuck(0);
            }

            if (gamepad1.dpad_up) {
                robot.spinArmBase(0.01);
                telemetry.addData("dpad", "up");
                sleep(15);
            } else if (gamepad1.dpad_down) {
                robot.spinArmBase(-0.01);
                telemetry.addData("dpad", "down");
                sleep(15);
            }

            if (gamepad1.a) { // Open
                robot.moveClaw(0.9);
            } else if (gamepad1.b) { // Close
                robot.moveClaw(0.2);
            }

            if (gamepad1.y) {
                robot.spinArmBase(0);
            }

            telemetry.addData("Servo amount", robot.armBase.getPosition());
            telemetry.addData("Servo amount claw", robot.claw.getPosition());
            telemetry.addData("Servo amount mirror", robot.clawMirror.getPosition());

            telemetry.update();
        }
    }
}
