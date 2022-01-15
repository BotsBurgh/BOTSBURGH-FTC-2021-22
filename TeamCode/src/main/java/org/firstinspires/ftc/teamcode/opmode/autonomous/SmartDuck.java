package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@Autonomous(name = "Smart Duck", group = Naming.OPMODE_GROUP_COMP)
public class SmartDuck extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        DistanceSensor sensorL = hardwareMap.get(DistanceSensor.class, "distance_left");
        DistanceSensor sensorR = hardwareMap.get(DistanceSensor.class, "distance_right");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        double distance = (sensorL.getDistance(DistanceUnit.CM) + sensorR.getDistance(DistanceUnit.CM)) / 2;

        // Move to duck wheel
        while (distance > 25 && opModeIsActive()) {
            robot.powerWheels(0.5);
            distance = (sensorL.getDistance(DistanceUnit.CM) + sensorR.getDistance(DistanceUnit.CM)) / 2;
        }

        robot.powerWheels(0);

        sleep(500);

        // Spin robot 180 degrees
        ElapsedTime runtime = new ElapsedTime();

        while (runtime.seconds() < 2 && opModeIsActive()) {
            robot.powerWheels(0.5, -0.5, 0.5, -0.5);
        }

        robot.powerWheels(0);

        sleep(500);

        // Power duck wheel for 15 seconds
        runtime = new ElapsedTime();

        while (runtime.seconds() < 15 && opModeIsActive()) {
            robot.powerDuck(0.7);
        }

        robot.powerDuck(0);

        sleep(500);

        runtime = new ElapsedTime();

        while (runtime.seconds() < 1 && opModeIsActive()) {
            robot.powerWheels(-1, 1, 1, -1);
        }

        robot.powerWheels(0);

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
