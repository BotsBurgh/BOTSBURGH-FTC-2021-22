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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.InitRobot;
import org.firstinspires.ftc.teamcode.api.OldRobot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@TeleOp(name = "Forward!", group = Naming.OPMODE_GROUP_TEST)
public class Forward extends LinearOpMode {
    @Override
    public void runOpMode() {
        InitRobot.init(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.x) {
                OldRobot.movement.move1x4(1);
            } else if (gamepad1.y) {
                OldRobot.movement.move1x4(0.3);
            } else if (gamepad1.a) {
                OldRobot.movement.move1x4(-0.2);
            } else {
                OldRobot.movement.move1x4(0);
            }
        }
    }
}
