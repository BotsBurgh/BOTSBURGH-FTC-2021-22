package org.firstinspires.ftc.teamcode.api.v2.bp;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * <code>AbstractRobot</code> is a class that defines methods that all robot-based classes
 * should use. It is handy for different variations of a robot. Eg. a prototype and the final
 * product.
 */
public abstract class AbstractRobot {
    /**
     * This base constructor for initializing a new <code>AbstractRobot</code>. It will only be used if
     * the <code>OpMode</code> does not exist yet.
     * creation.
     */
    public AbstractRobot() {}

    /**
     * This base constructor for initializing a new <code>AbstractRobot</code>. This is the most common
     * variant of this constructor, and should be used in basic <code>TeleOp</code>s.
     *
     * @param opMode The class the robot is going to be initialized for.
     */
    public AbstractRobot(OpMode opMode) {
        this.initTeleOp(opMode);
    }

    /**
     * This initializes all of the hardware necessary for the robot. It will only be used if
     * <code>AbstractRobot</code> is initialized without an <code>OpMode</code>.
     *
     * @param opMode The class the robot is going to be initialized for.
     */
    public abstract void initTeleOp(@NonNull OpMode opMode);

    /**
     * Move all of the motors necessary for transporting the robot forward.
     *
     * @param power The power that is applied to each motor.
     */
    public abstract void moveAll(double power);

    /**
     * Moves only the front motors.
     *
     * It is optional to implement this method because it assumes that the robot is rectangular
     * or uses a 4x4-based drive system.
     *
     * @param power The power that is applied to each motor.
     */
    public void moveFront(double power) {};

    /**
     * Moves only the back motors.
     *
     * It is optional to implement this method because it assumes that the robot is rectangular
     * or uses a 4x4-based drive system.
     *
     * @param power The power that is applied to each motor.
     */
    public void moveBack(double power) {};

    /**
     * Moves only the left motors.
     *
     * It is optional to implement this method because it assumes that the robot is rectangular
     * or uses a 4x4-based drive system.
     *
     * @param power The power that is applied to each motor.
     */
    public void moveLeft(double power) {};

    /**
     * Moves only the right motors.
     *
     * It is optional to implement this method because it assumes that the robot is rectangular
     * or uses a 4x4-based drive system.
     *
     * @param power The power that is applied to each motor.
     */
    public void moveRight(double power) {};
}
