package org.firstinspires.ftc.teamcode.api.hw;

import com.qualcomm.robotcore.hardware.AnalogInput;

import lombok.Setter;

public class Potentiometer {
    private final AnalogInput potentiometer;
    @Setter private double Vmax    = 0.004; // Minimum voltage
    @Setter private double Vmin    = 3.304; // Maximum voltage
    private double range = (7/4.0)*Math.PI; // Radians


    Potentiometer(AnalogInput potentiometer) {
        this.potentiometer = potentiometer;
    }

    /**
     * Gets the position (in degrees) of a potentiometer
     * @return Degrees of the potentiometer
     */
    public double positionRad() {
        return (range/(Vmax-Vmin))*(potentiometer.getVoltage()-Vmin);
    }

    public double positionDeg() {
        return positionRad()*(180/Math.PI);
    }

    public void setRangeDeg(double rangeDeg) {
        setRangeRad(rangeDeg*(Math.PI/180));
    }

    public void setRangeRad(double rangeRad) {
        this.range = rangeRad;
    }
}