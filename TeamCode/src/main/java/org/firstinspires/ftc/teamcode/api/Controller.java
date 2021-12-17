package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.bp.AbstractController;

public class Controller extends AbstractController {
    public Controller(LinearOpMode opMode, Robot robot, Gamepad gamepad) {
        super(opMode, robot, gamepad);
    }

    public Controller(LinearOpMode opMode, Robot robot) {
        super(opMode, robot);
    }

    public void joystick(float left_stick_x, float left_stick_y, float right_stick_x, float right_stick_y) {
        this.robot.frontLeft.setPower(Range.clip((-left_stick_y - right_stick_x), -1, 1));
        this.robot.frontRight.setPower(Range.clip((left_stick_y + right_stick_x), -1, 1));
        this.robot.backLeft.setPower(Range.clip((-left_stick_y - right_stick_x), -1, 1));
        this.robot.backRight.setPower(Range.clip((-left_stick_y + right_stick_x), -1, 1));
    }

    public void bumper(boolean left_bumper, boolean right_bumper) {
        if (left_bumper) {
            this.robot.spinDuck(1);
        } else if (right_bumper) {
            this.robot.spinDuck(-1);
        } else {
            this.robot.spinDuck(0);
        }
    }

    public void dpad(boolean dpad_up, boolean dpad_down, boolean dpad_left, boolean dpad_right) {
        if (dpad_up) {
            this.robot.spinArmBase(0.01);
        } else if (dpad_down) {
            this.robot.spinArmBase(-0.01);
        }

        if (dpad_left) {
            this.robot.moveClaw(0.2);
        } else if (dpad_right) {
            this.robot.moveClaw(-0.2);
        }
    }

}
