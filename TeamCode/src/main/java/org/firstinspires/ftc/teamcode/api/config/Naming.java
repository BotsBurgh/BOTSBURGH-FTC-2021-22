package org.firstinspires.ftc.teamcode.api.config;

/**
 * The Naming class makes it easier to prevent NullPointerExceptions when adding and getting items
 * from a HashMap or for Autonomous sides
 */
public class Naming {
    // Name configuration
    public static final String MOTOR_FL = "frontLeft";
    public static final String MOTOR_BL = "backLeft";
    public static final String MOTOR_FR = "frontRight";
    public static final String MOTOR_BR = "backRight";

    public static final String MOTOR_DUCK = "duck";

    public static final String SERVO_ARM_LEFT = "armLeft";
    public static final String SERVO_ARM_RIGHT = "armRight";

    public static final String SERVO_ARM_LEFTo = "armLefto";
    public static final String SERVO_ARM_RIGHTo = "armRighto";

    public static final String SERVO_CLAW_LEFT = "clawLeft";
    public static final String SERVO_CLAW_RIGHT = "clawRight";
  
    public static final String SERVO_CLAW_LEFTo = "clawLefto";
    public static final String SERVO_CLAW_RIGHTo = "clawRighto";

    public static final String SENSOR_DISTANCE_FL = "distanceFL";
    public static final String SENSOR_DISTANCE_FR = "distanceFR";
    public static final String SENSOR_DISTANCE_BL = "distanceBL";
    public static final String SENSOR_DISTANCE_BR = "distanceBR";

    public static final String SENSOR_COLOR_PARK = "parkSensor";

    public static final String GYRO_0 = "imu";
    public static final String GYRO_1 = "imu 1";

    public static final String WEBCAM_0 = "webcam";

    // OpMode group constants
    public static final String OPMODE_GROUP_COMP = "00-Competition"; // OpModes we would run during a competition
    public static final String OPMODE_GROUP_UTIL = "50-Utilities"; // OpModes that emit log output for calibration
    public static final String OPMODE_GROUP_DEMO = "60-Demos"; // Demo OpModes that are useless outside of showing off
    public static final String OPMODE_GROUP_TEST = "99-Test"; // Testing OpModes that move the robot
}
