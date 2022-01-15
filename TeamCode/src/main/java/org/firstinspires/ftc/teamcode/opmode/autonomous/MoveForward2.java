package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@Autonomous(name = "Move Forward 2", group = Naming.OPMODE_GROUP_COMP)
public class MoveForward2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        robot.powerWheels(1);
        sleep(2000);
        robot.powerWheels(0);

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
