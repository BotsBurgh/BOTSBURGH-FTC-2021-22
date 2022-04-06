package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Constants;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.Gyroscope;

@Autonomous(name = "Blue - Sensor Smart Duck", group = Naming.OPMODE_GROUP_COMP)
public class BlueSensorSmartDuck extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime runtime;

        Robot.setGyro0(new Gyroscope(hardwareMap.get(BNO055IMU.class, Naming.GYRO_0), "gyro0"));
        Robot.getGyro0().initGyro();
        Robot.getGyro0().calibrateGyro();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        robot.closeClaw();

        robot.getArmLeft().setPwmEnable();
        robot.getArmRight().setPwmEnable();
        robot.getClawLeft().setPwmEnable();
        robot.getClawRight().setPwmEnable();

        // Move left
        robot.powerWheels(0.5, -0.5, -0.5, 0.5);
        sleep(500);
        robot.powerWheels(0);
        sleep(500);

        // Move backwards into duck wheel
        runtime = new ElapsedTime();

        while (robot.getBackDistance() > 22 && runtime.seconds() < 5 && opModeIsActive()) {
            robot.powerWheels(-0.3);
        }

        robot.powerWheels(0);
        sleep(500);

        // Spin to wheel
        robot.gyroTurn(Constants.TURN_SPEED, -Math.PI / 4);
        robot.gyroHold(Constants.TURN_SPEED, -Math.PI / 4, 1);

        // Move back
        robot.powerWheels(-0.2);
        sleep(1000);
        robot.powerWheels(0);
        sleep(500);

        // Spin duck wheel
        runtime = new ElapsedTime();

        while (runtime.seconds() < 3 && opModeIsActive()) {
            robot.powerDuck(0.7);
        }

        robot.powerDuck(0);
        sleep(500);

        // Move forward
        robot.powerWheels(0.2);
        sleep(1000);
        robot.powerWheels(0);
        sleep(500);

        // Spin back
        robot.gyroTurn(Constants.TURN_SPEED, Math.PI / 12);
        robot.gyroHold(Constants.TURN_SPEED, Math.PI / 12, 1);

        return;

        /*// Spin duck wheel
        runtime = new ElapsedTime();

        while (runtime.seconds() < 3 && opModeIsActive()) {
            robot.powerDuck(-0.7);
        }

        robot.powerDuck(0);
        sleep(500);

        // Move left
        robot.powerWheels(0.5, -0.5, -0.5, 0.5);
        sleep(850);
        robot.powerWheels(0);
        sleep(500);

        // Raise Arm
        robot.positionArm(0.718);
        robot.positionArm2(0.718);
        sleep(500);

        // Move forward into warehouse
        runtime = new ElapsedTime();

        while (robot.getFRDistance() > 80 && runtime.seconds() < 5 && opModeIsActive()) {
            robot.powerWheels(1);
        }

        robot.powerWheels(0);

        // Done!
        telemetry.addData("Status", "Finished");
        telemetry.update();*/
    }
}
