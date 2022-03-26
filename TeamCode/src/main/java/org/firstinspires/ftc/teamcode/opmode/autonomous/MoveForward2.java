package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@Autonomous(name = "Move Forward 2", group = Naming.OPMODE_GROUP_COMP)
@Disabled
@Deprecated
public class MoveForward2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        // Move arm up
        robot.positionArm(0.718);

        // Move left a little bit
        ElapsedTime runtime = new ElapsedTime();

        while (runtime.seconds() < 0.4 && opModeIsActive()) {
            robot.powerWheels(-0.5, 0.5, 0.5, -0.5);
        }

        sleep(500);

        // Move forward
        runtime = new ElapsedTime();

        while (runtime.seconds() < 2 && opModeIsActive()) {
            robot.powerWheels(1);
        }

        robot.powerWheels(0);

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
