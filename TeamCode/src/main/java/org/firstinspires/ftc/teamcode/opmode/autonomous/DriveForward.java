package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@Autonomous(name = "Drive Straight Forward", group = Naming.OPMODE_GROUP_COMP)
public class DriveForward extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        ElapsedTime runtime;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        robot.closeClaw();
        robot.closeClaw2();

        robot.getArmLeft().setPwmEnable();
        robot.getArmRight().setPwmEnable();
        robot.getClawLeft().setPwmEnable();
        robot.getClawRight().setPwmEnable();

        // Move forward into warehouse
        runtime = new ElapsedTime();

        while (robot.getFRDistance() > 80 && runtime.seconds() < 5 && opModeIsActive()) {
            robot.powerWheels(1);
        }

        robot.powerWheels(0);

        // Done!
        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
