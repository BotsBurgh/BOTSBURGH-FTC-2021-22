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

        robot.reverseMotors(new DcMotor[] {robot.backRight});

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            double joystick_y = gamepad1.left_stick_y;
            double joystick_x = gamepad1.right_stick_x;

            robot.frontLeft.setPower(Range.clip((-joystick_y - joystick_x), -1, 1));
            robot.frontRight.setPower(Range.clip((joystick_y - joystick_x), -1, 1));
            robot.backLeft.setPower(Range.clip((-joystick_y - joystick_x), -1, 1));
            robot.backRight.setPower(Range.clip((-joystick_y + joystick_x), -1, 1));

            if (gamepad1.left_bumper) {
                robot.spinDuck(1);
            } else if (gamepad1.right_bumper) {
                robot.spinDuck(-1);
            } else {
                robot.spinDuck(0);
            }

            if (gamepad1.dpad_up) {
                robot.spinArmBase(0.01);
                telemetry.addData("dpad", "up");
            } else if (gamepad1.dpad_down) {
                robot.spinArmBase(-0.01);
                telemetry.addData("dpad", "down");
            } else {
                robot.spinArmBase(0);
                telemetry.addData("dpad", "none");
            }

            if (gamepad1.dpad_left) {
                robot.moveClaw(0.2);
            } else if (gamepad1.dpad_right) {
                robot.moveClaw(-0.2);
            } else {
                robot.moveClaw(0);
            }

            telemetry.update();
        }
    }
}
