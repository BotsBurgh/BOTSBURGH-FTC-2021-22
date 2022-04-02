package org.firstinspires.ftc.teamcode.api.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static int    THREADS          = 4;

    // Color sensor configuration
    public static double PARK_RED_FUDGE   = 250;
    public static double PARK_GREEN_FUDGE = 150;
    public static double PARK_BLUE_FUDGE  = 150;

    // Drive Adjustments
    public static double MOTOR_BL_POWER_MOD = 1.00;
    public static double MOTOR_BR_POWER_MOD = 1.00;
    public static double MOTOR_FL_POWER_MOD = 1.00;
    public static double MOTOR_FR_POWER_MOD = 1.00;

    public static double     COUNTS_PER_MOTOR_REV    = 752;    // eg: TETRIX Motor Encoder
    public static double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    public static double     WHEEL_DIAMETER_INCHES   = 3.779528;     // For figuring circumference
    public static double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public static double     DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    public static double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.

    public static double     HEADING_THRESHOLD       = 0.02;      // As tight as we can make it with an integer gyro
    public static double     P_TURN_COEFF            = 1.5;     // Larger is more responsive, but also less stable
    public static double     P_DRIVE_COEFF           = 2;     // Larger is more responsive, but also less stable
}
