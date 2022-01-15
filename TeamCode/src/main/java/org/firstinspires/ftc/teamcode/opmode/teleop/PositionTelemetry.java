package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.api.GyroTracker;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Constants;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.Button;
import org.firstinspires.ftc.teamcode.api.hw.Encoder;
import org.firstinspires.ftc.teamcode.api.hw.Gyroscope;
import org.firstinspires.ftc.teamcode.api.hw.Potentiometer;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor;

import java.util.HashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/*
 * This is a simple program to reach the white line
 */
@Config
@TeleOp(name="Position Telemetry", group=Naming.OPMODE_GROUP_UTIL)
public class PositionTelemetry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot.executorService = Executors.newFixedThreadPool(Constants.THREADS);
        Robot.gyro1 = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu 1"), "gyro1");
        Robot.gyro1.initGyro();
        Robot.gyro1.calibrateGyro();

        GyroTracker.reset();

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            GyroTracker.loop();
            telemetry.addData(">", "Press stop");
            telemetry.addData("x", GyroTracker.currentPosition.x);
            telemetry.addData("y", GyroTracker.currentPosition.y);
            telemetry.addData("z", GyroTracker.currentPosition.z);
            telemetry.addData("vx", GyroTracker.currentVelocity.xVeloc);
            telemetry.addData("vy", GyroTracker.currentVelocity.yVeloc);
            telemetry.addData("vz", GyroTracker.currentVelocity.zVeloc);
            telemetry.addData("ax", GyroTracker.currentAcceleration.xAccel);
            telemetry.addData("ay", GyroTracker.currentAcceleration.yAccel);
            telemetry.addData("az", GyroTracker.currentAcceleration.zAccel);
            telemetry.addData("avg_ax", GyroTracker.averageAcceleration.xAccel);
            telemetry.addData("avg_ay", GyroTracker.averageAcceleration.yAccel);
            telemetry.addData("avg_az", GyroTracker.averageAcceleration.zAccel);
            telemetry.update();
        }
    }
}
