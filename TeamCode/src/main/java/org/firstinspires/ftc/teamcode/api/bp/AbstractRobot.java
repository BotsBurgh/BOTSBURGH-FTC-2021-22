package org.firstinspires.ftc.teamcode.api.bp;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class AbstractRobot {
    // For now we don't need a non-argument constructor

    public AbstractRobot(LinearOpMode opMode) {
        this.initTeleOp(opMode);
    }

    public abstract void initTeleOp(@NonNull LinearOpMode opMode);
}
