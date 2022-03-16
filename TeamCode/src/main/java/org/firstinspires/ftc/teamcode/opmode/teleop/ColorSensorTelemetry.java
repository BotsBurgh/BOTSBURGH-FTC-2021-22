package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.Button;
import org.firstinspires.ftc.teamcode.api.hw.Encoder;
import org.firstinspires.ftc.teamcode.api.hw.Gyroscope;
import org.firstinspires.ftc.teamcode.api.hw.Potentiometer;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor;
import org.firstinspires.ftc.teamcode.api.Robot;

import java.util.HashMap;

/*
 * This is a simple program to reach the white line
 */
@Config
@TeleOp(name="Color Sensor Telemetry", group=Naming.OPMODE_GROUP_UTIL)
public class ColorSensorTelemetry extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SmartColorSensor sensor;

        double redFudge, greenFudge, blueFudge;
        int red, green, blue;


        telemetry.addLine();
        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData(">", "Press stop");

            for (String key : Sensor.colorSensors.keySet()) {
                sensor = Sensor.colorSensors.get(key);

                red = sensor.getRed();
                green = sensor.getGreen();
                blue = sensor.getBlue();

                float[] hsv = new float[3];
                Color.RGBToHSV(red, green, blue, hsv);

                telemetry.addData(key + " Red", red);
                telemetry.addData(key + " Green", green);
                telemetry.addData(key + " Blue", blue);
                telemetry.addData(key + " Hue", hsv[0]);
                telemetry.addData(key + " Saturation", hsv[1]);
                telemetry.addData(key + " Value", hsv[2]);
                telemetry.addData(key + " Detected", sensor.getRGB());
            }

            telemetry.update();
        }
    }
    static class Sensor {
        public static HashMap<String, Gyroscope> gyros; // Initialize gyroscopes
        public static HashMap<String, Potentiometer> pots; // Initialize potentiometers
        public static HashMap<String, Button> buttons; // Initialize buttons
        public static HashMap<String, SmartColorSensor> colorSensors; // Initialize color sensors
        public static HashMap<String, DistanceSensor> distances; // Initialize distance sensors
        public static HashMap<String, WebcamName> webcams; // Initialize webcams
        public static HashMap<String, Encoder> encoders; // Special encoders
    }
}
