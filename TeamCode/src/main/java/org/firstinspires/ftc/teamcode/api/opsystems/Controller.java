package org.firstinspires.ftc.teamcode.api.opsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.api.Robot;

public class Controller extends LinearOpMode {
    Robot robot = new Robot(this);

    protected void onJoystick(float leftStickX, float leftStickY, float rightStickX, float rightStickY) {}

    protected void onButton(boolean a, boolean b, boolean x, boolean y) {}

    protected void onBumper(boolean leftBumper, boolean rightBumper) {}

    protected void onDpad(boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight) { }

    @Override
    public void runOpMode() {
        Gamepad g = gamepad1;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");

        while (opModeIsActive()) {
            this.onJoystick(g.left_stick_x, g.left_stick_y, g.right_stick_x, g.right_stick_y);
            this.onButton(g.a, g.b, g.x, g.y);
            this.onBumper(g.left_bumper, g.right_bumper);
            this.onDpad(g.dpad_up, g.dpad_down, g.dpad_left, g.dpad_right);
            telemetry.update();
        }
    }
}
