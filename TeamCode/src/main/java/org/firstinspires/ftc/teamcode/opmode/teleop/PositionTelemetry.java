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
        Robot.gyro0 = new Gyroscope(hardwareMap.get(BNO055IMU.class, "imu"), "gyro0");
        Robot.gyro0.initGyro();
        Robot.gyro0.calibrateGyro();

        //Robot.gyro0.startAccelerationIntegration(new Position(), new Velocity(), 50);
        Acceleration acceleration;
        Velocity velocity;
        Position position;

        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            GyroTracker.loop();
            acceleration = GyroTracker.currentAcceleration;
            velocity = GyroTracker.currentVelocity;
            position = GyroTracker.currentPosition;
            telemetry.addData(">", "Press stop");
            telemetry.addData("x", position.x);
            telemetry.addData("y", position.y);
            telemetry.addData("z", position.z);
            telemetry.addData("vx", velocity.xVeloc);
            telemetry.addData("vy", velocity.yVeloc);
            telemetry.addData("vz", velocity.zVeloc);
            telemetry.addData("ax", acceleration.xAccel);
            telemetry.addData("ay", acceleration.yAccel);
            telemetry.addData("az", acceleration.zAccel);
            telemetry.update();
        }
    }
}
