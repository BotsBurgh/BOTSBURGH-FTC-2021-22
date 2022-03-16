package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@Autonomous(name = "Sensor Smart Duck", group = Naming.OPMODE_GROUP_COMP)
public class SensorSmartDuck extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        ElapsedTime runtime;

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

        // Move backwards into duck wheel

        runtime = new ElapsedTime();

        while (robot.getBackDistance() > 20 && runtime.seconds() < 5 && opModeIsActive()) {
            robot.powerWheels(-0.3);
        }

        robot.powerWheels(0);

        sleep(500);

        // Spin duck wheel

        runtime = new ElapsedTime();

        while (runtime.seconds() < 3 && opModeIsActive()) {
            robot.powerDuck(-0.7);
        }

        robot.powerDuck(0);

        sleep(500);

        // Move left

        runtime = new ElapsedTime();

        while (runtime.seconds() < 0.85 && opModeIsActive()) {
            robot.powerWheels(-0.5, 0.5, 0.5, -0.5);
        }

        robot.powerWheels(0);

        sleep(500);

        // Raise Arm

        robot.positionArm(0.718);

        sleep(500);

        // Move forward into warehouse

        runtime = new ElapsedTime();

        while (robot.getFRDistance() > 80 && runtime.seconds() < 5 && opModeIsActive()) {
            robot.powerWheels(1);
        }

        robot.powerWheels(0);

        // Finalize

        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
