package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@Autonomous(name = "Timed Smart Duck", group = Naming.OPMODE_GROUP_COMP)
@Disabled
@Deprecated
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

        robot.getArmLeft().setPwmEnable();
        robot.getArmRight().setPwmEnable();
        robot.getClawLeft().setPwmEnable();
        robot.getClawRight().setPwmEnable();

        // Move left a little bit
        robot.powerWheels(-0.5, 0.5, 0.5, -0.5);
        sleep(250);
        robot.powerWheels(0);
        sleep(500);

        // Move to backwards
        robot.powerWheels(-0.5);
        sleep(2250);
        robot.powerWheels(0);
        sleep(500);

        // Power duck wheel
        robot.powerDuck(-0.7);
        sleep(3000);
        robot.powerDuck(0);
        sleep(500);

        // Move left
        robot.powerWheels(-0.5, 0.5, 0.5, -0.5);
        sleep(750);
        robot.powerWheels(0);
        sleep(500);

        // Raise arm
        robot.positionArm(0.718);
        sleep(500);

        // Move to warehouse
        while (
                (sensorL.getDistance(DistanceUnit.CM) + sensorR.getDistance(DistanceUnit.CM)) / 2 > 90
                        && opModeIsActive()
        ) {
            robot.powerWheels(1);
        }

        robot.powerWheels(0);

        // Done!
        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
