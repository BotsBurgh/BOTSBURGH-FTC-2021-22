package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.vars.Group;

@Autonomous(name = "Depot", group = Group.CHALLENGE)
public class Depot extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        robot.moveAll(1);
        sleep(5000);
        robot.moveAll(0);
    }
}