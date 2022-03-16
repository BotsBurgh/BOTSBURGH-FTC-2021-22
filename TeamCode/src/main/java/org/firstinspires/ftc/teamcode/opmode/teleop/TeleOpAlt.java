package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.opsystems.Controller2Players;

/**
 * This is an example of extending the controller class.
 * Eventually this class will be deleted once TeleOpMain has been updated.
 * -BD103
 */
@TeleOp(name = "TeleOp Alt", group = Naming.OPMODE_GROUP_COMP)
public class TeleOpAlt extends Controller2Players {
    Robot robot = new Robot(this);

    /*
    * Controller 1
    *
    * It can adjust:
    *
    * - The core position (forward, backward, strafe, turn)
    * - The duck wheel (shared)
    * - The front arm
    *
    * Controller 2
    *
    * It can adjust:
    *
    * - The back arm
    * - The duck wheel (shared)
    */

    @Override

    protected void onJoystick(float leftStickX1, float leftStickY1, float rightStickX1, float rightStickY1) {
        double x1 = Math.pow(leftStickX1, 1.8);
        double y1 = Math.pow(leftStickY1, 1.8);
        double rotation = Math.pow(rightStickX1, 1.8);

        double flPower = Range.clip((x1 - y1 + rotation), -1.0, 1.0);
        double blPower = Range.clip((-x1 - y1 + rotation), -1.0, 1.0);
        double brPower = Range.clip((x1 - y1 - rotation), -1.0, 1.0);
        double frPower = Range.clip((-x1 - y1 - rotation), -1.0, 1.0);

        this.robot.powerWheels(flPower, frPower, blPower, brPower);
    }

    @Override
    protected void onButton(boolean a1, boolean b1, boolean x1, boolean y1) {
        if (x1) {
            robot.openClaw();
        } else if (b1) {
            robot.closeClaw();
        }

        if (y1) {
            this.robot.armLeft.setPosition(Range.clip(robot.armLeft.getPosition() - 0.001, 0.6, 1));
            this.robot.armRight.setPosition(Range.clip(robot.armRight.getPosition() - 0.001, 0.6, 1));
        } else if (a1) {
            this.robot.armLeft.setPosition(Range.clip(robot.armLeft.getPosition() + 0.001, 0.6, 1));
            this.robot.armRight.setPosition(Range.clip(robot.armRight.getPosition() + 0.001, 0.6, 1));
        }
    }

    @Override
    protected void onBumper(boolean leftBumper1, boolean rightBumper1, boolean leftBumper2, boolean rightBumper2) {
        if (leftBumper1 || leftBumper2) {
            this.robot.powerDuck(0.7);
        } else if (rightBumper1 || rightBumper2) {
            this.robot.powerDuck(-0.7);
        } else {
            this.robot.powerDuck(0);
        }
    }

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
                    g1.right_stick_y
            );
            this.onButton(g1.a, g1.b, g1.x, g1.y);
            this.onBumper(
                    g1.left_bumper,
                    g1.right_bumper,
                    g2.left_bumper,
                    g2.right_bumper
            );

            telemetry.update();
        }
    }
}
