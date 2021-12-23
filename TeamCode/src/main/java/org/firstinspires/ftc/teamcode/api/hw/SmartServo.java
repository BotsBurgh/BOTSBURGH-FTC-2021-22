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
 *
 * Inspiration from Out of the Box FTC Team 7244
 * (https://github.com/OutoftheBoxFTC/SkyStone/blob/EeshwarTesting/TeamCode/src/main/java/HardwareSystems/HardwareDevices/SmartServo.java)
 */

package org.firstinspires.ftc.teamcode.api.hw;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.api.OldRobot;

import lombok.Getter;
import lombok.Setter;

public class SmartServo {
    private final ServoImplEx servo;
    @Getter private double position;
    @Getter @Setter private double step = 0.001;
    @Getter private boolean lock = false;

    // Motor configuration
    @Getter @Setter double maxPos = 1;
    @Getter @Setter double minPos = 0;

    public SmartServo(Servo servo){
        this.servo = (ServoImplEx) servo;
        position = 0;
        this.servo.setPwmDisable();
    }

    public void setPosition(double position) {
        if (this.position != position) {
            this.position = position;
            if (position <= 1) {
                servo.setPosition(position);
            } else {
                if (!servo.isPwmEnabled()) {
                    servo.setPwmEnable();
                }

                if (this.position != position) {
                    double min = servo.getPwmRange().usPulseLower;
                    double max = servo.getPwmRange().usPulseUpper;
                    servo.setPosition(((position-min)/max)*(maxPos-minPos)+minPos);
                }
            }
        }
    }

    public void disableServo() {
        if (servo.isPwmEnabled()) {
            servo.setPwmDisable();
        } else {
            servo.setPwmEnable();
        }
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    /**
     * Scan the servo (move the servo slowly) to a position.
     * @param position Position (in on a scale of  0-1) to scan the servo to.
     * @param ms Time to scan
     */
    private void scanServo(double position, int ms) {
        boolean direction = position > servo.getPosition();
        long lastLoopStart = System.nanoTime();
        int sleep;
        while (Math.abs(servo.getPosition() - position) < 0.001) {
            sleep = (int)(position/ms - Math.abs(lastLoopStart - System.nanoTime())*1000);
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

    public void scanServoAsync(double position, int ms) {
        if (lock) {
            return;
        }
        lock = true;
        OldRobot.executor.execute(new Runnable() {
            @Override
            public void run() {
                scanServo(position, ms);
            }
        });
        lock = false;
    }

    public void scanServoSync(double position, int ms) {
        if (lock) {
            return;
        }
        lock = true;
        scanServo(position, ms);
        lock = false;
    }
}