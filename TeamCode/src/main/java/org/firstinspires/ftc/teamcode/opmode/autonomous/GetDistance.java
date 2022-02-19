package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@Autonomous(name = "Get Distance", group= Naming.OPMODE_GROUP_UTIL)
public class GetDistance extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("Front Average Distance", robot.getFrontDistance());
            telemetry.addData("Front Left Distance", robot.getFLDistance());
            telemetry.addData("Front Right Distance", robot.getFRDistance());

            telemetry.addData("Back Average Distance", robot.getBackDistance());
            telemetry.addData("Back Left Distance", robot.getBLDistance());
            telemetry.addData("Back Right Distance", robot.getBRDistance());

            telemetry.update();
        }
    }
}
