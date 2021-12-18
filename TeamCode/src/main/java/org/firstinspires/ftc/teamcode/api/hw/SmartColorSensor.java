package org.firstinspires.ftc.teamcode.api.hw;

import static android.graphics.Color.RGBToHSV;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import lombok.Getter;
import lombok.Setter;

public class SmartColorSensor {
    private final NormalizedColorSensor sensor;
    @Setter private double blackThresh  = 0.2;
    @Setter private double whiteThresh  = 0.9;

    public int getRed() {
        return (int) Range.clip(this.getNormalizedColors().red * 255 * redFudge, 0, 255);
    }

    public int getGreen() {
        return (int) Range.clip(this.getNormalizedColors().green * 255 * greenFudge, 0, 255);
    }

    public int getBlue() {
        return (int) Range.clip(this.getNormalizedColors().blue * 255 * blueFudge, 0, 255);
    }

    public enum Color {
        RED,
        ORANGE,
        YELLOW,
        GREEN,
        BLUE,
        PURPLE,
        WHITE,
        GRAY,
        BLACK,
        BROWN
    }

    public SmartColorSensor(NormalizedColorSensor sensor) {
        this.sensor = sensor;
    }

    // Color sensor config
    @Getter @Setter double redFudge, greenFudge, blueFudge = 1;

    public NormalizedRGBA getNormalizedColors() {
        return sensor.getNormalizedColors();
    }

    public Color getRGB() {
        float[] hsv = new float[3];
        RGBToHSV(
                (int) Range.clip(this.getNormalizedColors().red * 255 * redFudge, 0, 255),
                (int) Range.clip(this.getNormalizedColors().green * 255 * greenFudge, 0, 255),
                (int) Range.clip(this.getNormalizedColors().blue * 255 * blueFudge, 0, 255),
                hsv
        );

        if (hsv[1] < 0.2) {
            // Greyscale (inner core in HSV cylinder)
            if (hsv[2] > whiteThresh) {
                return Color.WHITE;
            } else if (hsv[2] < blackThresh) {
                return Color.BLACK;
            }
        }

        // If the value is too low, black
        if (hsv[2] < blackThresh) {
            return Color.BLACK;
        } else {
            if ((hsv[0] > 320) || (hsv[0] <= 20)) {
                return Color.RED;
            } else if ((hsv[0] > 20) && (hsv[0] <= 46)) {
                return Color.ORANGE;
            } else if ((hsv[0] > 46) && (hsv[0] <= 64)) {
                return Color.YELLOW;
            } else if ((hsv[0] > 146) && (hsv[0] <= 160)) {
                return Color.GREEN;
            } else if ((hsv[0] > 160) && (hsv[0] <= 248)) {
                return Color.BLUE;
            } else if ((hsv[0] > 248) && (hsv[0] <= 320)) {
                return Color.PURPLE;
            }
        }
        return Color.GRAY;
    }
}