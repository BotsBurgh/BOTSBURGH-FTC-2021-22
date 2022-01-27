package org.firstinspires.ftc.teamcode.api.opsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller2Players extends Controller {
    protected void onJoystick(
            float leftStickX1,
            float leftStickY1,
            float rightStickX1,
            float rightStickY1,
            float leftStickX2,
            float leftStickY2,
            float rightStickX2,
            float rightStickY2
    ) {}

    protected void onButton(
            boolean a1,
            boolean b1,
            boolean x1,
            boolean y1,
            boolean a2,
            boolean b2,
            boolean x2,
            boolean y2
    ) {}

    protected void onBumper(
            boolean leftBumper1,
            boolean rightBumper1,
            boolean leftBumper2,
            boolean rightBumper2
    ) {}

    protected void onDpad(
            boolean dpadUp1,
            boolean dpadDown1,
            boolean dpadLeft1,
            boolean dpadRight1,
            boolean dpadUp2,
            boolean dpadDown2,
            boolean dpadLeft2,
            boolean dpadRight2
    ) { }

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
            this.onJoystick(
                    g1.left_stick_x,
                    g1.left_stick_y,
                    g1.right_stick_x,
                    g1.right_stick_y,
                    g2.left_stick_x,
                    g2.left_stick_y,
                    g2.right_stick_x,
                    g2.right_stick_y
            );
            this.onButton(
                    g1.a,
                    g1.b,
                    g1.x,
                    g1.y,
                    g2.a,
                    g2.b,
                    g2.x,
                    g2.y
            );
            this.onBumper(
                    g1.left_bumper,
                    g1.right_bumper,
                    g2.left_bumper,
                    g2.right_bumper
            );
            this.onDpad(
                    g1.dpad_up,
                    g1.dpad_down,
                    g1.dpad_left,
                    g1.dpad_right,
                    g2.dpad_up,
                    g2.dpad_down,
                    g2.dpad_left,
                    g2.dpad_right
            );

            telemetry.update();
        }
    }
}
