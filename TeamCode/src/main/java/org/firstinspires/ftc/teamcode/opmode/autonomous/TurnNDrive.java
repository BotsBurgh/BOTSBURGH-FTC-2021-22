package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Constants;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.Gyroscope;

@Autonomous(name = "Turn and Drive", group = Naming.OPMODE_GROUP_DEMO)
public class TurnNDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot.setGyro0(new Gyroscope(hardwareMap.get(BNO055IMU.class, Naming.GYRO_0), "gyro0"));
        Robot.getGyro0().initGyro();
        Robot.getGyro0().calibrateGyro();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        robot.gyroTurn(Constants.TURN_SPEED, -Math.PI/4);
        robot.gyroHold(Constants.TURN_SPEED, -Math.PI/4, 0.5);
        robot.gyroTurn(Constants.TURN_SPEED, Math.PI/4);
        robot.gyroHold(Constants.TURN_SPEED, Math.PI/4, 0.5);
        robot.gyroTurn(Constants.TURN_SPEED, 0.0);
        robot.gyroDrive(Constants.DRIVE_SPEED, 48.0, 0.0);

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}

