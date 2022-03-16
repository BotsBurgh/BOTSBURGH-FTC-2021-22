package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.api.GyroTracker;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Constants;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.Gyroscope;
import org.firstinspires.ftc.teamcode.api.hw.SmartServo;

import java.util.concurrent.Executors;

/*
 * This is a simple program to reach the white line
 */
@Config
@TeleOp(name="Async Servo Tester", group=Naming.OPMODE_GROUP_UTIL)
public class AsyncServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot.executorService = Executors.newFixedThreadPool(Constants.THREADS);

        SmartServo servo = new SmartServo(hardwareMap.get(ServoImplEx.class, "testServo"));

        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        //servo.scanServoAsync(1, 10000);
        servo.scanServoSync(1, 10000);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData(">", "Press stop");
            telemetry.addData("Time", System.currentTimeMillis());
            telemetry.addData("Locked", servo.isLocked());
            telemetry.addData("Position", servo.getPosition());
            telemetry.update();
        }
    }
}
