package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.opsystems.Controller;

/**
 * This is an example of extending the controller class.
 * Eventually this class will be deleted once TeleOpMain has been updated.
 * -BD103
 */
@TeleOp(name = "TeleOp Alt", group = Naming.OPMODE_GROUP_COMP)
public class TeleOpAlt extends Controller {
    Robot robot = new Robot(this);

    @Override
    protected void onJoystick(float leftStickX, float leftStickY, float rightStickX, float rightStickY) {
        double x1 = leftStickX;
        double y1 = leftStickY;
        double rotation = rightStickX;

        double flPower = Range.clip(( x1 - y1 + rotation), -1.0, 1.0);
        double blPower = Range.clip((-x1 - y1 + rotation), -1.0, 1.0);
        double brPower = Range.clip(( x1 - y1 - rotation), -1.0, 1.0);
        double frPower = Range.clip((-x1 - y1 - rotation), -1.0, 1.0);

        this.robot.powerWheels(flPower, frPower, blPower, brPower);
    }

    @Override
    protected void onButton(boolean a, boolean b, boolean x, boolean y) {
        if (a) {
            robot.openClaw();
        } else if (b) {
            robot.closeClaw();
        }

        if (x) {
            this.robot.powerStepWheels(1);
        } else if (y) {
            this.robot.powerStepWheels(-1);
        } else {
            this.robot.powerStepWheels(0);
        }
    }

    @Override
    protected void onBumper(boolean leftBumper, boolean rightBumper) {
        if (leftBumper) {
            this.robot.powerDuck(0.7);
        } else if (rightBumper) {
            this.robot.powerDuck(-0.7);
        } else {
            this.robot.powerDuck(0);
        }
    }

    @Override
    protected void onDpad(boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight) {
        if (dpadUp) {
            this.robot.armLeft.setPosition(Range.clip(robot.armLeft.getPosition() - 0.001, 0.6, 1));
            this.robot.armRight.setPosition(Range.clip(robot.armRight.getPosition() - 0.001, 0.6, 1));
        } else if (dpadDown) {
            this.robot.armLeft.setPosition(Range.clip(robot.armLeft.getPosition() + 0.001, 0.6, 1));
            this.robot.armRight.setPosition(Range.clip(robot.armRight.getPosition() + 0.001, 0.6, 1));
        }
    }
}
