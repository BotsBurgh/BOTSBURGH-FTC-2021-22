package org.firstinspires.ftc.teamcode.api.hw;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class Gyroscope {
    private final BNO055IMU gyro;
    private final String calibrationFile;

    public Acceleration getAcceleration() {
        return gyro.getAcceleration();
    }

    public Acceleration getGravity() {
        return gyro.getGravity();
    }

    public Acceleration getLinearAcceleration() {
        return gyro.getLinearAcceleration();
    }

    public Orientation getAngularOrientation() {
        return gyro.getAngularOrientation();
    }

    public Gyroscope(BNO055IMU gyro, String calibrationFile) {
        this.gyro = gyro;
        this.calibrationFile = calibrationFile;
    }

    /**
     * Calibrates a gyroscope
     */
    public void calibrateGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        gyro.initialize(parameters);

        BNO055IMU.CalibrationData calibrationData = gyro.readCalibrationData();
        File file = AppUtil.getInstance().getSettingsFile(calibrationFile);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }

    /**
     * Initializes the gyroscope.
     */
    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = calibrationFile;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);
    }
}
