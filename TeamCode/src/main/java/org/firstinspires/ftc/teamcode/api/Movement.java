package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashMap;
import java.util.Objects;

public class Movement {
    public static HashMap<String, DcMotorEx> motors;

    public void move(double flPower, double frPower, double blPower, double brPower){
        Objects.requireNonNull(motors.get(Constants.MOTOR_FL)).setPower(flPower);
        Objects.requireNonNull(motors.get(Constants.MOTOR_FR)).setPower(frPower);
        Objects.requireNonNull(motors.get(Constants.MOTOR_BL)).setPower(blPower);
        Objects.requireNonNull(motors.get(Constants.MOTOR_BR)).setPower(brPower);
    }

    public void move(double power) {
        move(power, power, power, power);
    }
}
