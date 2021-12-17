package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.api.Controller;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.vars.Group;

@TeleOp(name = "Move with Controller 2", group = Group.UTILITIES)
public class MoveController2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        Controller controller = new Controller(this, robot);

        robot.backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        controller.run();
    }
}
