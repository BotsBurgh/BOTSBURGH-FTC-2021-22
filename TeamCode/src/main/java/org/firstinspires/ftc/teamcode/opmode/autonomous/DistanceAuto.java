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

        boolean running = true;
        double distance = 100;

        while (distance > 10) {
            double distanceL = sensorL.getDistance(DistanceUnit.CM);
            double distanceR = sensorR.getDistance(DistanceUnit.CM);
            distance = (distanceL + distanceR) / 2;

            robot.powerWheels(0.5);

            telemetry.addData("Average Distance", distance);
            telemetry.addData("Left Distance", distanceL);
            telemetry.addData("Right Distance", distanceR);
            telemetry.update();
        }

        robot.powerDuck(0);
    }
}
