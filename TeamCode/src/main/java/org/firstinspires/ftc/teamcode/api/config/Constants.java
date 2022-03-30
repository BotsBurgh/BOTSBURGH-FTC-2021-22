package org.firstinspires.ftc.teamcode.api.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static final int    THREADS          = 4;

    // Color sensor configuration
    public static final double PARK_RED_FUDGE   = 250;
    public static final double PARK_GREEN_FUDGE = 150;
    public static final double PARK_BLUE_FUDGE  = 150;

    // Drive Adjustments
    public static final double MOTOR_BL_POWER_MOD = 1.00;
    public static final double MOTOR_BR_POWER_MOD = 1.00;
    public static final double MOTOR_FL_POWER_MOD = 1.00;
    public static final double MOTOR_FR_POWER_MOD = 1.00;

    public static final double     COUNTS_PER_MOTOR_REV    = 752;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 3.779528;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public static final double     DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    public static final double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.

    public static final double     HEADING_THRESHOLD       = 1;      // As tight as we can make it with an integer gyro
    public static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
}
