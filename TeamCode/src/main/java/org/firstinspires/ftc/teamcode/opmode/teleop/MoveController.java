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
            double joystick_x = gamepad1.left_stick_x;
            double joystick_y = gamepad1.right_stick_y;

            robot.frontLeft.setPower(Range.clip((-joystick_x + joystick_y), -1, 1));
            robot.frontRight.setPower(Range.clip((-joystick_x - joystick_y), -1, 1));
            robot.backLeft.setPower(Range.clip((-joystick_x + joystick_y), -1, 1));
            robot.backRight.setPower(Range.clip((-joystick_x - joystick_y), -1, 1));

            if (gamepad1.left_bumper) {
                robot.spinDuck(1);
            } else if (gamepad1.right_bumper) {
                robot.spinDuck(-1);
            } else {
                robot.spinDuck(0);
            }

            if (gamepad1.dpad_up) {
                robot.spinArmBase(1);
            } else if (gamepad1.dpad_down) {
                robot.spinArmBase(-1);
            } else {
                robot.spinArmBase(0);
            }
        }
    }
}
