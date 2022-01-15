package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.Robot;

@Autonomous(name = "Distance Auto")
public class DistanceAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        DistanceSensor sensorL = hardwareMap.get(DistanceSensor.class, "distance_left");
        DistanceSensor sensorR = hardwareMap.get(DistanceSensor.class, "distance_right");

        waitForStart();

        while (opModeIsActive()) {
            double distanceL = sensorL.getDistance(DistanceUnit.CM);
            double distanceR = sensorR.getDistance(DistanceUnit.CM);
            double distance = (distanceL + distanceR) / 2;

            telemetry.addData("Average Distance", distance);
            telemetry.addData("Left Distance", distanceL);
            telemetry.addData("Right Distance", distanceR);
            telemetry.update();
        }
    }
}
