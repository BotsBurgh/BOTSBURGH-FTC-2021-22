package org.firstinspires.ftc.teamcode.api;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import lombok.Getter;

public class GyroTracker {
    public static Acceleration gravity = new Acceleration(DistanceUnit.METER, 0, 0, 0,0);
    public static Velocity currentVelocity = new Velocity(DistanceUnit.METER, 0, 0, 0, 0);
    public static Acceleration currentAcceleration = new Acceleration(DistanceUnit.METER, 0, 0, 0, 0);
    public static Position currentPosition = new Position(DistanceUnit.METER, 0, 0, 0, 0);

    @Getter
    private static long prev = System.currentTimeMillis();

    public static void loop() {
        currentAcceleration.xAccel = Robot.gyro0.getLinearAcceleration().xAccel - gravity.xAccel;
        currentAcceleration.yAccel = Robot.gyro0.getLinearAcceleration().yAccel - gravity.yAccel;
        currentAcceleration.zAccel = Robot.gyro0.getLinearAcceleration().zAccel - gravity.zAccel;

        currentVelocity.xVeloc += currentAcceleration.xAccel * (System.currentTimeMillis() - prev) / 1000.0;
        currentVelocity.yVeloc += currentAcceleration.yAccel * (System.currentTimeMillis() - prev) / 1000.0;
        currentVelocity.zVeloc += currentAcceleration.zAccel * (System.currentTimeMillis() - prev) / 1000.0;

        currentPosition.x += currentVelocity.xVeloc * (System.currentTimeMillis() - prev) / 1000.0;
        currentPosition.y += currentVelocity.yVeloc * (System.currentTimeMillis() - prev) / 1000.0;
        currentPosition.z += currentVelocity.zVeloc * (System.currentTimeMillis() - prev) / 1000.0;

        prev = System.currentTimeMillis();
    }
}