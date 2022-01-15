package org.firstinspires.ftc.teamcode.api;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import lombok.Getter;

public class GyroTracker {
    public static Velocity currentVelocity = new Velocity(DistanceUnit.METER, 0, 0, 0, 0);
    public static Acceleration currentAcceleration;
    public static Position currentPosition = new Position(DistanceUnit.METER, 0, 0, 0, 0);

    private static long prev = System.currentTimeMillis();

    public static void loop() {
        currentAcceleration = Robot.gyro0.getAcceleration();
        currentVelocity.xVeloc += currentAcceleration.xAccel * (System.currentTimeMillis() - prev) / 1000000.0;
        currentVelocity.yVeloc += currentAcceleration.yAccel * (System.currentTimeMillis() - prev) / 1000000.0;
        currentVelocity.zVeloc += currentAcceleration.zAccel * (System.currentTimeMillis() - prev) / 1000000.0;
        currentPosition.x += currentVelocity.xVeloc * (System.currentTimeMillis() - prev) / 1000000.0;
        currentPosition.y += currentVelocity.yVeloc * (System.currentTimeMillis() - prev) / 1000000.0;
        currentPosition.z += currentVelocity.zVeloc * (System.currentTimeMillis() - prev) / 1000000.0;
        prev = System.currentTimeMillis();
    }
}