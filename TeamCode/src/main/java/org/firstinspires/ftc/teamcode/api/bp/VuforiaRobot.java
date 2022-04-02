package org.firstinspires.ftc.teamcode.api.bp;

import org.firstinspires.ftc.teamcode.api.util.DuckPos;

public interface VuforiaRobot {
    public default void initTfod() {
    }

    default void initVuforia() {
    }

    default DuckPos getDuckPos() {
        return DuckPos.NONE;
    }
}
