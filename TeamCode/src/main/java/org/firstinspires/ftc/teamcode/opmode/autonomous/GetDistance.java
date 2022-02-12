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

        DistanceSensor sensorFL = robot.getDistanceFL();
        DistanceSensor sensorFR = robot.getDistanceFR();
        DistanceSensor sensorBL = robot.getDistanceBL();
        DistanceSensor sensorBR = robot.getDistanceBR();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            double distanceFL = sensorFL.getDistance(DistanceUnit.CM);
            double distanceFR = sensorFR.getDistance(DistanceUnit.CM);
            double distanceF = (distanceFL + distanceFR) / 2;

            double distanceBL = sensorBL.getDistance(DistanceUnit.CM);
            double distanceBR = sensorBR.getDistance(DistanceUnit.CM);
            double distanceB = (distanceBL + distanceBR) / 2;

            telemetry.addData("Front Average Distance", distanceF);
            telemetry.addData("Front Left Distance", distanceFL);
            telemetry.addData("Front Right Distance", distanceFR);

            telemetry.addData("Back Average Distance", distanceB);
            telemetry.addData("Back Left Distance", distanceBL);
            telemetry.addData("Back Right Distance", distanceBR);

            telemetry.update();
        }
    }
}
