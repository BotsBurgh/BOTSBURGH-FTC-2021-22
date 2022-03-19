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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.api.bp.AbstractRobot;
import org.firstinspires.ftc.teamcode.api.bp.ArmRobot;
import org.firstinspires.ftc.teamcode.api.bp.DistanceSensorRobot;
import org.firstinspires.ftc.teamcode.api.bp.WheeledRobot;
import org.firstinspires.ftc.teamcode.api.config.Constants;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.Gyroscope;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor;
import org.firstinspires.ftc.teamcode.api.hw.SmartMotor;
import org.firstinspires.ftc.teamcode.api.hw.SmartServo;

import java.util.concurrent.ExecutorService;

import lombok.Getter;
import lombok.Setter;

public class Robot extends AbstractRobot implements WheeledRobot, ArmRobot, DistanceSensorRobot {
    public static ExecutorService executorService;

    @Getter
    private SmartMotor bl, br, fl, fr, duck;
    @Getter
    private SmartServo armLeft, armRight, clawLeft, clawRight, clawLeft2, clawRight2, armLeft2, armRight2;
    @Getter
    private DistanceSensor distanceFL, distanceFR, distanceBL, distanceBR;
    @Getter
    private SmartColorSensor parkSensor;
    @Getter
    private WebcamName webcam0;
    @Getter
    @Setter
    private static Gyroscope gyro0, gyro1;
    @Getter
    private OpMode opMode;

    public Robot(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void powerWheels(double flPower, double frPower, double blPower, double brPower) {
        this.fl.setPower(flPower);
        this.fr.setPower(frPower);
        this.bl.setPower(blPower);
        this.br.setPower(brPower);
    }

    // Arm 1

    @Override
    public void positionArm(double position) {
        this.armLeft.setPosition(position);
        this.armRight.setPosition(position);
    }

    @Override
    public void positionClaw(double position) {
        this.clawLeft.setPosition(position);
        this.clawRight.setPosition(position);
    }

    public void adjustArm(double amount) {
        this.armLeft.setPosition(this.armLeft.getPosition() + amount);
        this.armRight.setPosition(this.armRight.getPosition() + amount);
    }

    @Override
    public void adjustClaw(double amount) {
        this.clawLeft.setPosition(this.clawLeft.getPosition() + amount);
        this.clawRight.setPosition(this.clawRight.getPosition() + amount);
    }

    // Arm 2

    public void positionArm2(double position) {
        this.armLeft2.setPosition(position);
        this.armRight2.setPosition(position);
    }

    public void positionClaw2(double position) {
        this.clawLeft2.setPosition(position);
        this.clawRight2.setPosition(position);
    }

    public void adjustArm2(double amount) {
        this.armLeft2.setPosition(this.armLeft2.getPosition() + amount);
        this.armRight2.setPosition(this.armRight2.getPosition() + amount);
    }

    public void adjustClaw2(double amount) {
        this.clawLeft2.setPosition(this.clawLeft2.getPosition() + amount);
        this.clawRight2.setPosition(this.clawRight2.getPosition() + amount);
    }

    @Override
    public void openClaw() {
        this.positionClaw(0.1);
    }

    @Override
    public void closeClaw() {
        this.positionClaw(0.2);
    }

    public void openClaw2() {
        this.positionClaw2(0.1);
    }

    public void closeClaw2() {
        this.positionClaw2(0.2);
    }

    @Override
    public void powerDuck(double power) {
        this.duck.setPower(power);
    }

    @Override
    public double getFLDistance() {
        return this.distanceFL.getDistance(DistanceUnit.CM);
    }

    @Override
    public double getFRDistance() {
        return this.distanceFR.getDistance(DistanceUnit.CM);
    }

    @Override
    public double getBLDistance() {
        return this.distanceBL.getDistance(DistanceUnit.CM);
    }

    @Override
    public double getBRDistance() {
        return this.distanceBR.getDistance(DistanceUnit.CM);
    }

    @Override
    public void initTeleOp(@NonNull OpMode opMode) {
        this.opMode = opMode;

        // Motors
        this.bl = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BL));
        this.br = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BR));
        this.fl = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FL));
        this.fr = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FR));

        this.duck = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_DUCK));

        // Servos
        this.armLeft = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_LEFT));
        this.armRight = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_RIGHT));
        this.clawLeft = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_LEFT));
        this.clawRight = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_RIGHT));

        this.armLeft2 = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_LEFTo));
        this.armRight2 = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_RIGHTo));
        this.clawLeft2 = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_LEFTo));
        this.clawRight2 = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_RIGHTo));

        // Sensors
        this.distanceFL = opMode.hardwareMap.get(DistanceSensor.class, Naming.SENSOR_DISTANCE_FL);
        this.distanceFR = opMode.hardwareMap.get(DistanceSensor.class, Naming.SENSOR_DISTANCE_FR);
        this.distanceBL = opMode.hardwareMap.get(DistanceSensor.class, Naming.SENSOR_DISTANCE_BL);
        this.distanceBR = opMode.hardwareMap.get(DistanceSensor.class, Naming.SENSOR_DISTANCE_BR);

        // Looped Config
        SmartMotor[] wheelLoop = new SmartMotor[]{this.fl, this.fr, this.bl, this.br};
        SmartServo[] servoLoop = new SmartServo[]{this.armLeft, this.armRight, this.clawLeft, this.clawRight};

        for (SmartMotor motor : wheelLoop) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (SmartServo servo : servoLoop) {
            servo.setPosition(servo.getPosition());
        }

        // Specific Config
        this.fl.setDirection(DcMotor.Direction.REVERSE);
        this.fr.setDirection(DcMotor.Direction.REVERSE);
        this.bl.setDirection(DcMotor.Direction.REVERSE);
        this.br.setDirection(DcMotor.Direction.FORWARD);

        this.armLeft.setDirection(Servo.Direction.FORWARD);
        this.armRight.setDirection(Servo.Direction.REVERSE);
        this.clawLeft.setDirection(Servo.Direction.FORWARD);
        this.clawRight.setDirection(Servo.Direction.REVERSE);

        this.armLeft2.setDirection(Servo.Direction.FORWARD);
        this.armRight2.setDirection(Servo.Direction.REVERSE);
        this.clawLeft2.setDirection(Servo.Direction.FORWARD);
        this.clawRight2.setDirection(Servo.Direction.REVERSE);

        this.bl.setPowerModifier(Constants.MOTOR_BL_POWER_MOD);
        this.br.setPowerModifier(Constants.MOTOR_BR_POWER_MOD);
        this.fl.setPowerModifier(Constants.MOTOR_FL_POWER_MOD);
        this.fr.setPowerModifier(Constants.MOTOR_FR_POWER_MOD);
    }
}
