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

package org.firstinspires.ftc.teamcode.API;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.HW.SmartMotor;
import org.firstinspires.ftc.teamcode.API.HW.SmartServo;

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
    public void move4x4(double flPower, double frPower, double blPower, double brPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_FL)).setPower(flPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_FR)).setPower(frPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL)).setPower(blPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR)).setPower(brPower);
    }

    /**
     * Moves each of the four motors individually. Best for mecanum drives.
     * @param power Power to all the wheels
     */
    public void move1x4(double power) {
        move4x4(power,power,power,power);
    }


    /**
     * Move the base with four motors based on individual sides, not wheels
     * @param lPower Power to the left side
     * @param rPower Power to the right side
     */
    public void move2x4(double lPower, double rPower) {
        move4x4(lPower, rPower, lPower, rPower);
    }

    /**
     * Move the base with two motors and two values
     * @param lPower Power sent to back left motor
     * @param rPower Power sent to back right motor
     */
    public void move2x2(double lPower, double rPower) {
        Objects.requireNonNull(motors.get(Naming.MOTOR_BL)).setPower(lPower);
        Objects.requireNonNull(motors.get(Naming.MOTOR_BR)).setPower(rPower);
    }

    // TODO: Javadoc
    public void move1x2(double power) {
        move2x2(power, power);
    }

    /**
     * Scan the servo (move the servo slowly) to a position.
     * @param id ID of servo
     * @param degrees Position (in on a scale of  0-1) to scan the servo to.
     * @param ms Time to scan
     */
    public void scanServo(String id, double degrees, boolean direction, int ms) {
        // Run in another thread
        Robot.executor.execute(new Runnable() {
            @Override
            public void run() {
                SmartServo servo = Objects.requireNonNull(servos.get(id));
                double step = servo.getStep();
                long lastLoopStart = System.nanoTime();
                int sleep;
                while (Math.abs(servo.getPosition() - degrees) < 0.001) {
                    sleep = (int)(degrees/ms - Math.abs(lastLoopStart - System.nanoTime())*1000);
                    if (direction) {
                        // Scan up
                        servo.setPosition(servo.getPosition() + step);
                    } else {
                        // Scan down
                        servo.setPosition(servo.getPosition() - step);
                    }
                    SystemClock.sleep(sleep);
                }
            }
        });
    }
}