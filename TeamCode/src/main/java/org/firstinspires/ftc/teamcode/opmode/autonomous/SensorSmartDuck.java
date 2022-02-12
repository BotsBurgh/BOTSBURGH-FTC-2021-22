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
        DistanceSensor sensorFL = robot.getDistanceFL();
        DistanceSensor sensorFR = robot.getDistanceFR();
        DistanceSensor sensorBR = robot.getDistanceBR();
        DistanceSensor sensorBL = robot.getDistanceBL();

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

        double distance = sensorBR.getDistance(DistanceUnit.CM);

        while (distance > 30 && opModeIsActive()) {
            robot.powerWheels(-0.3);
            distance = sensorBR.getDistance(DistanceUnit.CM);
        }

        robot.powerWheels(0);

        sleep(500);

        // Spin duck wheel

        ElapsedTime runtime = new ElapsedTime();

        while (runtime.seconds() < 3 && opModeIsActive()) {
            robot.powerDuck(-0.7);
        }

        robot.powerDuck(0);

        sleep(500);
    }
}
