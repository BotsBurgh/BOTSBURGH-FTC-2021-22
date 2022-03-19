package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Laser Test", group = "Sensor")
public class LaserTest extends LinearOpMode{

    private double distance;

    @Override
    public void runOpMode() throws InterruptedException {

        DistanceSensor sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            distance = sensorRange.getDistance(DistanceUnit.CM);
            telemetry.update();
            telemetry.addData("deviceName",sensorRange.getDeviceName() );
            telemetry.addData("range", distance);

            if(distance < 10 ){
                telemetry.addData("Distance", "YOU ARE GONNA HIT THE WALL");
            }
            else{
                telemetry.addData("Distance", "You are fine");
            }
        }
    }
}
