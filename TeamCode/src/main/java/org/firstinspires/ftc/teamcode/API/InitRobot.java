package org.firstinspires.ftc.teamcode.API;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.API.Config.Constants;
import org.firstinspires.ftc.teamcode.API.Config.Naming;
import org.firstinspires.ftc.teamcode.API.HW.SmartColorSensor;
import org.firstinspires.ftc.teamcode.API.HW.SmartMotor;
import org.firstinspires.ftc.teamcode.API.HW.SmartServo;
import org.firstinspires.ftc.teamcode.API.HW.Gyroscope;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Objects;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * The Robot Initializer. Place initialization code here. This prevents needing to sync the init code
 * between all OpModes.
 */
public class InitRobot {

    // TODO: JavaDoc
    public static void init(@NotNull OpMode l) {
        ExecutorService executorService = Executors.newFixedThreadPool(Constants.THREADS);

        /*
        * #######                   ######
        * #       #####  # #####    #     # ###### #       ####  #    #
        * #       #    # #   #      #     # #      #      #    # #    #
        * #####   #    # #   #      ######  #####  #      #    # #    #
        * #       #    # #   #      #     # #      #      #    # # ## #
        * #       #    # #   #      #     # #      #      #    # ##  ##
        * ####### #####  #   #      ######  ###### ######  ####  #    #
        */

        // Get motors
        SmartMotor bl = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BL));
        SmartMotor br = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BR));
        SmartMotor fl = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FL));
        SmartMotor fr = new SmartMotor(l.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FR));

        Movement.motors = new HashMap<>();
        Movement.motors.put(Naming.MOTOR_BL, bl);
        Movement.motors.put(Naming.MOTOR_BR, br);
        Movement.motors.put(Naming.MOTOR_FL, fl);
        Movement.motors.put(Naming.MOTOR_FR, fr);

        // Get servos
        SmartServo armBaseLeft = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_ARM_LEFT));
        SmartServo armBaseRight = new SmartServo(l.hardwareMap.get(Servo.class, Naming.SERVO_ARM_RIGHT));

        // Add servos into the list
        Movement.servos = new HashMap<>();
        Movement.servos.put(Naming.SERVO_ARM_LEFT, armBaseLeft);
        Movement.servos.put(Naming.SERVO_ARM_RIGHT, armBaseRight);

        // Add CRServos into the list
        Movement.crServos = new HashMap<>();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to spin in the correct direction
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setPowerModifier(Constants.MOTOR_BL_POWER_MOD);
        br.setPowerModifier(Constants.MOTOR_BR_POWER_MOD);
        fl.setPowerModifier(Constants.MOTOR_FL_POWER_MOD);
        fr.setPowerModifier(Constants.MOTOR_FR_POWER_MOD);

        // Get color sensors
        SmartColorSensor parkSensor = new SmartColorSensor(
                (NormalizedColorSensor)l.hardwareMap.get(ColorSensor.class,
                        Naming.COLOR_SENSOR_PARK)
        );
        parkSensor.setRedFudge(Constants.PARK_RED_FUDGE);
        parkSensor.setGreenFudge(Constants.PARK_GREEN_FUDGE);
        parkSensor.setBlueFudge(Constants.PARK_BLUE_FUDGE);

        // Add color sensors into list
        Sensor.colorSensors = new HashMap<>();
        Sensor.colorSensors.put(Naming.COLOR_SENSOR_PARK, parkSensor);

        // Get webcams
        WebcamName webcam1 = l.hardwareMap.get(WebcamName.class, Naming.WEBCAM_0);

        // Add webcams to list
        Sensor.webcams = new HashMap<>();
        Sensor.webcams.put(Naming.WEBCAM_0, webcam1);

        // Get gyros
        Gyroscope gyro0 = new Gyroscope(l.hardwareMap.get(BNO055IMU.class, Naming.GYRO_0), Naming.GYRO_0);
        Gyroscope gyro1 = new Gyroscope(l.hardwareMap.get(BNO055IMU.class, Naming.GYRO_1), Naming.GYRO_1);

        // Add gyros to list
        Sensor.gyros = new HashMap<>();
        Sensor.gyros.put(Naming.GYRO_0, gyro0);
        Sensor.gyros.put(Naming.GYRO_1, gyro1);

        Sensor.encoders = new HashMap<>();

        Robot.opMode = l;

        // Send power to servos so they don't move
        for (String key : Movement.servos.keySet()) {
            SmartServo servo = Objects.requireNonNull(Movement.servos.get(key));
            servo.setPosition(servo.getPosition());
        }

        // Initialize gyros
        for (String key : Sensor.gyros.keySet()) {
            Objects.requireNonNull(Sensor.gyros.get(key)).initGyro();
        }
    }
}