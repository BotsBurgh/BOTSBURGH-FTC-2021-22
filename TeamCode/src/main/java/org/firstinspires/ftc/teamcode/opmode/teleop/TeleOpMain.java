/*
Copyright 2020 FIRST Tech Challenge Team 11792
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@TeleOp(name = "TeleOp Main", group = Naming.OPMODE_GROUP_COMP)
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

            robot.powerWheels(flPower, frPower, blPower, brPower);

            if (gamepad1.left_bumper) {
                robot.powerDuck(0.7);
            } else if (gamepad1.right_bumper) {
                robot.powerDuck(-0.7);
            } else {
                robot.powerDuck(0);
            }

            if (gamepad1.dpad_up) {
                //robot.armLeft.scanServoSync(robot.armLeft.getPosition() + 0.01, 20);
                //robot.armRight.scanServoSync(robot.armRight.getPosition() + 0.01, 20);
                robot.armLeft.setPosition(Range.clip(robot.armLeft.getPosition() - 0.01, 0.6, 1));
                robot.armRight.setPosition(Range.clip(robot.armRight.getPosition() - 0.01, 0.6, 1));
            } else if (gamepad1.dpad_down) {
                //robot.armLeft.scanServoSync(robot.armLeft.getPosition() - 0.01, 20);
                //robot.armRight.scanServoSync(robot.armRight.getPosition() - 0.01, 20);
                robot.armLeft.setPosition(Range.clip(robot.armLeft.getPosition() + 0.01, 0.6, 1));
                robot.armRight.setPosition(Range.clip(robot.armRight.getPosition() + 0.01, 0.6, 1));
            }

            if (gamepad1.a) {
                // Open
                robot.openClaw();
            } else if (gamepad1.b) {
                // Close
                robot.closeClaw();
            }

            if (gamepad1.x) {
                robot.left.setPower(1);
                robot.right.setPower(1);
            } else {
                robot.left.setPower(0);
                robot.right.setPower(0);
            }

            telemetry.addData("Arm Left", robot.armLeft.getPosition());
            telemetry.addData("Arm Right", robot.armRight.getPosition());
            telemetry.addData("Claw Left", robot.clawLeft.getPosition());
            telemetry.addData("Claw Right", robot.clawRight.getPosition());

            telemetry.update();
        }
    }
}
