package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Random;

/**
 * @author BD103
 */
@TeleOp(name = "MEMS", group = "random")
public class MovingTeleOp extends LinearOpMode {
    private static Random rand = new Random();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (rand.nextBoolean()) {
                telemetry.addData("Status", "Running");
                telemetry.update();
            } else {
                telemetry.addData("Status", "Woo");
                telemetry.update();
            }
        }
    }
}
