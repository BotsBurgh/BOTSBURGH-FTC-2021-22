package org.firstinspires.ftc.teamcode.api.opsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller2Players extends Controller {
    protected void onJoystick2(float leftStickX, float leftStickY, float rightStickX, float rightStickY) {}

    protected void onButton2(boolean a, boolean b, boolean x, boolean y) {}

    protected void onBumper2(boolean leftBumper, boolean rightBumper) {}

    protected void onDpad2(boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight) { }

    @Override
    public void runOpMode() {
        Gamepad g1 = gamepad1;
        Gamepad g2 = gamepad2;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");

        while (opModeIsActive()) {
            // Controller 1
            this.onJoystick(g1.left_stick_x, g1.left_stick_y, g1.right_stick_x, g1.right_stick_y);
            this.onButton(g1.a, g1.b, g1.x, g1.y);
            this.onBumper(g1.left_bumper, g1.right_bumper);
            this.onDpad(g1.dpad_up, g1.dpad_down, g1.dpad_left, g1.dpad_right);

            // Controller 2
            this.onJoystick2(g2.left_stick_x, g2.left_stick_y, g2.right_stick_x, g2.right_stick_y);
            this.onButton2(g2.a, g2.b, g2.x, g2.y);
            this.onBumper2(g2.left_bumper, g2.right_bumper);
            this.onDpad2(g2.dpad_up, g2.dpad_down, g2.dpad_left, g2.dpad_right);

            telemetry.update();
        }
    }
}
