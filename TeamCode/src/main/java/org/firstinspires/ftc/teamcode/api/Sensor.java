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

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.api.hw.Button;
import org.firstinspires.ftc.teamcode.api.hw.Encoder;
import org.firstinspires.ftc.teamcode.api.hw.Gyroscope;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor;
import org.firstinspires.ftc.teamcode.api.hw.Potentiometer;

import java.util.HashMap;

/**
 * The Sensor class.
 * This is a class which interfaces with sensors, so you don't have to. See the README for more
 * information.
 */
public class Sensor {
    public static HashMap<String, Gyroscope> gyros; // Initialize gyroscopes
    public static HashMap<String, Potentiometer> pots; // Initialize potentiometers
    public static HashMap<String, Button> buttons; // Initialize buttons
    public static HashMap<String, SmartColorSensor> colorSensors; // Initialize color sensors
    public static HashMap<String, DistanceSensor> distances; // Initialize distance sensors
    public static HashMap<String, WebcamName> webcams; // Initialize webcams
    public static HashMap<String, Encoder> encoders; // Special encoders
}