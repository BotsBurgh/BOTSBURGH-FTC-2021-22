package org.firstinspires.ftc.teamcode.api;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import lombok.Getter;

public class GyroTracker {
    public static Velocity currentVelocity = new Velocity(DistanceUnit.METER, 0, 0, 0, System.nanoTime());
    public static Acceleration currentAcceleration;
    public static Acceleration totalAcceleration = new Acceleration(DistanceUnit.METER, 0, 0, 0, System.nanoTime());
    public static Acceleration averageAcceleration = new Acceleration(DistanceUnit.METER, 0, 0, 0, System.nanoTime());
    public static Position currentPosition = new Position(DistanceUnit.METER, 0, 0, 0, System.nanoTime());
    public static Acceleration offset = new Acceleration(DistanceUnit.METER, 0.1545, -0.00215, -0.006, System.nanoTime());

    private static long count = 1;

    @Getter
    private static long prev = System.currentTimeMillis();

    public static void reset() {
        currentVelocity = new Velocity(DistanceUnit.METER, 0, 0, 0, 0);
        currentPosition = new Position(DistanceUnit.METER, 0, 0, 0, 0);
        totalAcceleration = new Acceleration(DistanceUnit.METER, 0, 0, 0, 0);
        averageAcceleration = new Acceleration(DistanceUnit.METER, 0, 0, 0, 0);
    }

    public static void loop() {
        currentAcceleration = Robot.getGyro1().getLinearAcceleration();
        totalAcceleration.xAccel += currentAcceleration.xAccel;
        totalAcceleration.yAccel += currentAcceleration.yAccel;
        totalAcceleration.zAccel += currentAcceleration.zAccel;
        averageAcceleration.xAccel = totalAcceleration.xAccel / count;
        averageAcceleration.yAccel = totalAcceleration.yAccel / count;
        averageAcceleration.zAccel = totalAcceleration.zAccel / count;

        currentVelocity.xVeloc += (currentAcceleration.xAccel - offset.xAccel) * (System.currentTimeMillis() - prev) / 1000.0;
        currentVelocity.yVeloc += (currentAcceleration.yAccel - offset.yAccel) * (System.currentTimeMillis() - prev) / 1000.0;
        currentVelocity.zVeloc += (currentAcceleration.zAccel - offset.zAccel) * (System.currentTimeMillis() - prev) / 1000.0;

        currentPosition.x += currentVelocity.xVeloc * (System.currentTimeMillis() - prev) / 1000.0;
        currentPosition.y += currentVelocity.yVeloc * (System.currentTimeMillis() - prev) / 1000.0;
        currentPosition.z += currentVelocity.zVeloc * (System.currentTimeMillis() - prev) / 1000.0;

        prev = System.currentTimeMillis();
        count += 1;
    }
}