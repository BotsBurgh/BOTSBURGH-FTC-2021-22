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

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.InitRobot;
import org.firstinspires.ftc.teamcode.api.Movement;
import org.firstinspires.ftc.teamcode.api.Robot;

import java.util.Objects;

@TeleOp(name = "TeleOp Main", group = Naming.OPMODE_GROUP_COMP)
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() {
        InitRobot.init(this);

        MultipleTelemetry telemetry = new MultipleTelemetry();

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

            Movement.move4x4(flPower, frPower, blPower, brPower);

            if (gamepad1.left_bumper) {
                Robot.spinDuck(0.7);
            } else if (gamepad1.right_bumper) {
                Robot.spinDuck(-0.7);
            } else {
                Robot.spinDuck(0);
            }

            if (gamepad1.dpad_up) {
                Objects.requireNonNull(Movement.servos.get(Naming.SERVO_ARM_LEFT)).scanServoAsync(
                        Objects.requireNonNull(Movement.servos.get(Naming.SERVO_ARM_LEFT)).getPosition() + 0.01, 20
                );
                Objects.requireNonNull(Movement.servos.get(Naming.SERVO_ARM_RIGHT)).scanServoAsync(
                        Objects.requireNonNull(Movement.servos.get(Naming.SERVO_ARM_RIGHT)).getPosition() + 0.01, 20
                );
            } else if (gamepad1.dpad_down) {
                Objects.requireNonNull(Movement.servos.get(Naming.SERVO_ARM_LEFT)).scanServoAsync(
                        Objects.requireNonNull(Movement.servos.get(Naming.SERVO_ARM_LEFT)).getPosition() - 0.01, 20
                );
                Objects.requireNonNull(Movement.servos.get(Naming.SERVO_ARM_RIGHT)).scanServoAsync(
                        Objects.requireNonNull(Movement.servos.get(Naming.SERVO_ARM_RIGHT)).getPosition() - 0.01, 20
                );
            }

            if (gamepad1.a) { // Open
                Robot.openClaw();
            } else if (gamepad1.b) { // Close
                Robot.closeClaw();
            }

            telemetry.update();
        }
    }
}
