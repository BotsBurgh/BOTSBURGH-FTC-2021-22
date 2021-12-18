/*
Copyright 2020 FIRST Tech Challenge Team 11792
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode.api;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.SmartMotor;
import org.firstinspires.ftc.teamcode.api.hw.SmartServo;

import java.util.HashMap;
import java.util.Objects;

import lombok.Builder;

/**
 * The Movement class. Interfaces with servos and motors so you don't have to
 */
@Builder
public class Movement {
    public static HashMap<String, SmartMotor> motors;
    public static HashMap<String, SmartServo> servos;
    public static HashMap <String, CRServo> crServos;

    /**
     * Moves each of the four motors individually. Best for mecanum drives.
     * @param flPower Power to the front left wheel
     * @param frPower Power to the front right wheel
     * @param blPower Power to the back left wheel
     * @param brPower Power to the back right wheel
     */
    public static void move4x4(double flPower, double frPower, double blPower, double brPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_FL)).setPower(flPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_FR)).setPower(frPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL)).setPower(blPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR)).setPower(brPower);
    }

    /**
     * Moves each of the four motors individually. Best for mecanum drives.
     * @param power Power to all the wheels
     */
    public static void move1x4(double power) {
        move4x4(power,power,power,power);
    }


    /**
     * Move the base with four motors based on individual sides, not wheels
     * @param lPower Power to the left side
     * @param rPower Power to the right side
     */
    public static void move2x4(double lPower, double rPower) {
        move4x4(lPower, rPower, lPower, rPower);
    }

    /**
     * Move the base with two motors and two values
     * @param lPower Power sent to back left motor
     * @param rPower Power sent to back right motor
     */
    public static void move2x2(double lPower, double rPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL)).setPower(lPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR)).setPower(rPower);
    }

    // TODO: Javadoc
    public static void move1x2(double power) {
        move2x2(power, power);
    }
}