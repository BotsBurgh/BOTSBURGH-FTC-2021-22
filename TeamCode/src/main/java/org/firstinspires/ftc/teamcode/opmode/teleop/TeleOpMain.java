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

        robot.positionClaw(0.1);
        robot.positionClaw2(0.1);

        while (opModeIsActive()) {
            // Wheel movement
            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            double flPower = Range.clip((x1 - y1 + rotation), -1.0, 1.0);
            double blPower = Range.clip((-x1 - y1 + rotation), -1.0, 1.0);
            double brPower = Range.clip((x1 - y1 - rotation), -1.0, 1.0);
            double frPower = Range.clip((-x1 - y1 - rotation), -1.0, 1.0);

            robot.powerWheels(flPower, frPower, blPower, brPower);

            // Duck wheel logic
            if ((gamepad1.left_bumper || gamepad2.left_bumper) && !(gamepad1.right_bumper || gamepad2.right_bumper)) {
                robot.powerDuck(0.7);
            } else if ((gamepad1.right_bumper || gamepad2.right_bumper) && !(gamepad1.left_bumper || gamepad2.left_bumper)) {
                robot.powerDuck(-0.7);
            } else {
                robot.powerDuck(0);
            }

            // Arm movement
            if (gamepad1.dpad_up) {
                robot.positionArm2(Range.clip(robot.getArmLeft2().getPosition() - 0.01, 0.5, 1));
            } else if (gamepad1.dpad_down) {
                robot.positionArm2(Range.clip(robot.getArmLeft2().getPosition() + 0.01, 0.5, 1));
            }

            if (gamepad2.dpad_up) {
                robot.positionArm(Range.clip(robot.getArmLeft().getPosition() - 0.01, 0.4, 1));
            } else if (gamepad2.dpad_down) {
                robot.positionArm(Range.clip(robot.getArmLeft().getPosition() + 0.01, 0.4, 1));
            }

            // Claw opening / closing
            if (gamepad1.a) {
                robot.openClaw2();
            } else if (gamepad1.b) {
                robot.closeClaw2();
            }

            if (gamepad2.a) {
                robot.openClaw();
            } else if (gamepad2.b) {
                robot.closeClaw();
            }
        }
    }
}
