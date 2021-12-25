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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor.Color;

import java.util.Objects;
import java.util.concurrent.Executor;

import lombok.Builder;

/**
 * Integrates Sensor class and Movement class so we can use VuForia and motors in one function.
 * Use like so: Robot robot = new Robot(new Sensor(whatever), new Movement(whatever));
 * Refer to Movement and Sensor classes for more information on what those classes do.
 * NOTE: You really should edit this file to suit your robot. If you find an error occurring here,
 * add it to our GitHub issues page at https://github.com/botsburgh/BOTSBURGH-FTC-2020-21/issues
 */
@Builder
public class Robot {
    public static Sensor sensor;
    public static Movement movement;
    public static OpMode opMode;
    public static Executor executor;

    public static void whiteLine(String sensor, double power) {
        driveToColor(sensor, power, Color.WHITE);
    }

    public static void driveToColor(String sensor, double power, Color targetColor) {
        while (true) {
            Color color = Sensor.colorSensors.get(sensor).getRGB();
            if (color == targetColor) {
                Movement.move1x4(0);
                break;
            } else {
                Movement.move1x4(power);
            }
        }
    }

    public static void spinDuck(double power) {
        Objects.requireNonNull(Movement.motors.get(Naming.MOTOR_DUCK)).setPower(power);
    }

    public static void openClaw() {
        Objects.requireNonNull(Movement.servos.get(Naming.SERVO_CLAW_LEFT)).setPosition(0.9);
        Objects.requireNonNull(Movement.servos.get(Naming.SERVO_CLAW_RIGHT)).setPosition(0.9);
    }
    public static void closeClaw() {
        Objects.requireNonNull(Movement.servos.get(Naming.SERVO_CLAW_LEFT)).setPosition(0.2);
        Objects.requireNonNull(Movement.servos.get(Naming.SERVO_CLAW_RIGHT)).setPosition(0.2);
    }
}