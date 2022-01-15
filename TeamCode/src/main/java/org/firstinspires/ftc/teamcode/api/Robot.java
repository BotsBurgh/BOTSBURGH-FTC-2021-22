/*
 * Copyright 2020 FIRST Tech Challenge Team 11792
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.api;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.api.bp.AbstractRobot;
import org.firstinspires.ftc.teamcode.api.bp.ArmRobot;
import org.firstinspires.ftc.teamcode.api.bp.WheeledRobot;
import org.firstinspires.ftc.teamcode.api.config.Constants;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.Gyroscope;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor;
import org.firstinspires.ftc.teamcode.api.hw.SmartMotor;
import org.firstinspires.ftc.teamcode.api.hw.SmartServo;

import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import lombok.Getter;

public class Robot extends AbstractRobot implements WheeledRobot, ArmRobot {
    public static ExecutorService executorService;

    // Discuss if private is better idea
    public SmartMotor bl, br, fl, fr, duck, left, right;
    public SmartServo armLeft, armRight, clawLeft, clawRight;
    public SmartColorSensor parkSensor;
    public WebcamName webcam0;
    public static Gyroscope gyro0, gyro1;
    private OpMode opMode;

    public Robot(OpMode opMode) {
        super(opMode);
    }

    // Wheels
    @Override
    public void powerWheels(double flPower, double frPower, double blPower, double brPower) {
        this.fl.setPower(flPower);
        this.fr.setPower(frPower);
        this.bl.setPower(blPower);
        this.br.setPower(brPower);
    }

    // Arm

    /*
    * The actual arm base uses asynchronous code, so that
    * will not be implemented until this gets tested
    */

    @Override
    public void positionClaw(double position) {
        this.clawLeft.setPosition(position);
        this.clawRight.setPosition(position);
    }

    @Override
    public void adjustClaw(double amount) {
        // Version 1
        this.clawLeft.setPosition(this.clawLeft.getPosition() + amount);
        this.clawRight.setPosition(this.clawRight.getPosition() + amount);

        // Version 2

        /*
        * double finalPosition = this.clawLeft.getPosition() + amount;
        * this.clawLeft.setPosition(finalPosition);
        * this.clawRight.setPosition(finalPosition);
        */
    }

    @Override
    public void openClaw() {
        this.positionClaw(0.9);
    }

    @Override
    public void closeClaw() {
        this.positionClaw(0.82);
    }

    @Override
    public void powerDuck(double power) {
        this.duck.setPower(power);
    }

    @Override
    public void initTeleOp(@NonNull OpMode opMode) {
        // Threading or something
        ExecutorService executorService = Executors.newFixedThreadPool(Constants.THREADS);

        // For future reference
        this.opMode = opMode;

        // InitRobot has an executor service, not sure if that should be included here

        // Motors
        this.bl = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BL));
        this.br = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BR));
        this.fl = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FL));
        this.fr = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FR));

        this.duck = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_DUCK));

        this.left = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_LEFT));
        this.right = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_RIGHT));

        // Servos
        this.armLeft = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_LEFT));
        this.armRight = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_RIGHT));
        this.clawLeft = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_LEFT));
        this.clawRight = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_RIGHT));

        /*
        // Sensors
        this.parkSensor = new SmartColorSensor(
                (NormalizedColorSensor) opMode.hardwareMap.get(ColorSensor.class, Naming.COLOR_SENSOR_PARK)
        );
        this.webcam0 = opMode.hardwareMap.get(WebcamName.class, Naming.WEBCAM_0);
        */
        gyro0 = new Gyroscope(opMode.hardwareMap.get(BNO055IMU.class, Naming.GYRO_0), Naming.GYRO_0);
        gyro1 = new Gyroscope(opMode.hardwareMap.get(BNO055IMU.class, Naming.GYRO_1), Naming.GYRO_1);

        // Looped Config
        SmartMotor[] wheelLoop = new SmartMotor[]{this.fl, this.fr, this.bl, this.br, this.right, this.left};
        SmartServo[] servoLoop = new SmartServo[]{this.armLeft, this.armRight, this.clawLeft, this.clawRight};
        Gyroscope[] gyroLoop = new Gyroscope[]{this.gyro0, this.gyro1};

        for (SmartMotor motor : wheelLoop) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (SmartServo servo : servoLoop) {
            servo.setPosition(servo.getPosition());
        }

        /*for (Gyroscope gyro : gyroLoop) {
            gyro.initGyro();
        }*/

        // Specific Config
        this.fl.setDirection(DcMotor.Direction.REVERSE);
        this.fr.setDirection(DcMotor.Direction.FORWARD);
        this.bl.setDirection(DcMotor.Direction.REVERSE);
        this.br.setDirection(DcMotor.Direction.FORWARD);
        this.right.setDirection(DcMotorSimple.Direction.REVERSE);
        this.left.setDirection(DcMotorSimple.Direction.FORWARD);

        this.armLeft.setDirection(Servo.Direction.FORWARD);
        this.armRight.setDirection(Servo.Direction.REVERSE);
        this.clawLeft.setDirection(Servo.Direction.FORWARD);
        this.clawRight.setDirection(Servo.Direction.REVERSE);

        this.bl.setPowerModifier(Constants.MOTOR_BL_POWER_MOD);
        this.br.setPowerModifier(Constants.MOTOR_BR_POWER_MOD);
        this.fl.setPowerModifier(Constants.MOTOR_FL_POWER_MOD);
        this.fr.setPowerModifier(Constants.MOTOR_FR_POWER_MOD);

        /*this.parkSensor.setRedFudge(Constants.PARK_RED_FUDGE);
        this.parkSensor.setGreenFudge(Constants.PARK_GREEN_FUDGE);
        this.parkSensor.setBlueFudge(Constants.PARK_BLUE_FUDGE);*/
    }
}
