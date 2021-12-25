package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.InitRobot;
import org.firstinspires.ftc.teamcode.api.Movement;
import org.firstinspires.ftc.teamcode.api.config.Naming;

@Autonomous(name = "Depot", group = Naming.OPMODE_GROUP_COMP)
public class Depot extends LinearOpMode {
    @Override
    public void runOpMode() {
        InitRobot.init(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        Movement.move1x4(1);
        sleep(2000);
        Movement.move1x4(0);
    }
}