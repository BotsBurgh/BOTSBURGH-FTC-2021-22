package org.firstinspires.ftc.teamcode.api.bp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * <code>AbstractSmartRobot</code> is a smart version of <code>AbstractRobot</code> that allows
 * higher-level control of a robot's functions.
 */
public abstract class AbstractSmartRobot extends AbstractRobot {
    public AbstractSmartRobot() {
        super();
    }

    public AbstractSmartRobot(OpMode opMode) {
        super(opMode);
    }

    public void smartMoveAll(double distance) {}
    public void smartMoveFront(double distance) {}
    public void smartMoveBack(double distance) {}
    public void smartMoveLeft(double distance) {}
    public void smartMoveRight(double distance) {}

    public void smartTurnLeft(double degrees) {}
    public void smartTurnRight(double degrees) {}
}
