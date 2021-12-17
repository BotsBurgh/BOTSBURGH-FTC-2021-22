package org.firstinspires.ftc.teamcode.api.bp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.apache.commons.math3.analysis.function.Abs;
import org.firstinspires.ftc.teamcode.api.Robot;

public class AbstractController {
    protected LinearOpMode opMode;
    protected Robot robot;
    protected Gamepad gamepad;
    
    public AbstractController(LinearOpMode opMode, Robot robot, Gamepad gamepad) {
        this.opMode = opMode;
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public AbstractController(LinearOpMode opMode, Robot robot) {
        this(opMode, robot, opMode.gamepad1);
    }

    public void joystick(float left_stick_x, float left_stick_y, float right_stick_x, float right_stick_y) {}

    public void bumper(boolean left_bumper, boolean right_bumper) {}

    public void dpad(boolean dpad_up, boolean dpad_down, boolean dpad_left, boolean dpad_right) {}

    protected void runEvent() {
        this.joystick(this.gamepad.left_stick_x, this.gamepad.left_stick_y, this.gamepad.right_stick_x, this.gamepad.right_stick_y);
        this.bumper(this.gamepad.left_bumper, this.gamepad.right_bumper);
        this.dpad(this.gamepad.dpad_up, this.gamepad.dpad_down, this.gamepad.dpad_left, this.gamepad.dpad_right);
        this.opMode.telemetry.update();
    }

    public void run() {
        while (opMode.opModeIsActive()) {
            runEvent();
        }
    }
}
