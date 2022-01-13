package org.firstinspires.ftc.teamcode.api.util;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.api.Robot;

public class TrackerUtil {
    private static boolean running = false;
    public static void startTracker(){
        running=true;
        Robot.executor.execute(new Runnable() {
            @Override
            public void run() {
                Acceleration currentAccel;
                Quaternion currentOrientation;
                while(running=true){
                    currentAccel = Robot.gyro0.getAcceleration();
                    currentOrientation = Robot.gyro0.getQuaternionOrientation();
                }
            }
        });

    }

    public static void stopTracker(){
        running=false;
    }

}
