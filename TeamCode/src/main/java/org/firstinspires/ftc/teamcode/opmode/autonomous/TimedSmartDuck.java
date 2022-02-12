package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@Autonomous(name = "Timed Smart Duck", group = Naming.OPMODE_GROUP_COMP)
public class TimedSmartDuck extends LinearOpMode {
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

        robot.closeClaw();

        robot.armLeft.setPwmEnable();
        robot.armRight.setPwmEnable();
        robot.clawLeft.setPwmEnable();
        robot.clawRight.setPwmEnable();

        // Move left a little bit
        ElapsedTime runtime = new ElapsedTime();

        while (runtime.seconds() < 0.25 && opModeIsActive()) {
            robot.powerWheels(-0.5, 0.5, 0.5, -0.5);
        }

        robot.powerWheels(0);

        // Move to backwards
        runtime = new ElapsedTime();

        while (runtime.seconds() < 2.25 && opModeIsActive()) {
            robot.powerWheels(-0.5);
        }

        robot.powerWheels(0);

        sleep(500);

        // Power duck wheel
        runtime = new ElapsedTime();

        while (runtime.seconds() < 3 && opModeIsActive()) {
            robot.powerDuck(-0.7);
        }

        robot.powerDuck(0);

        sleep(500);

        // Move left
        runtime = new ElapsedTime();

        while (runtime.seconds() < 0.75 && opModeIsActive()) {
            robot.powerWheels(-0.5, 0.5, 0.5, -0.5);
        }

        robot.powerWheels(0);

        sleep(500);

        // Raise arm
        robot.positionArm(0.718);

        sleep(500);

        // Move forward into warehouse
        runtime = new ElapsedTime();

        /* while (runtime.seconds() < 3 && opModeIsActive()) {
            robot.powerWheels(1);
            robot.powerStepWheels(1);
        } */

        while (
                (sensorL.getDistance(DistanceUnit.CM) + sensorR.getDistance(DistanceUnit.CM)) / 2 > 90
                && opModeIsActive()
        ) {
            robot.powerWheels(1);
            robot.powerStepWheels(1);
        }

        robot.powerWheels(0);

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}