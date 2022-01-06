package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor;
import org.firstinspires.ftc.teamcode.api.Robot;
import org.firstinspires.ftc.teamcode.api.Sensor;

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
}
