package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.api.config.Naming;

@TeleOp(name = "Audio Concept", group = Naming.OPMODE_GROUP_DEMO)
public class AudioConcept extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean soundPlaying = false;
        boolean soundFound = false;

        int soundID = hardwareMap.appContext.getResources().getIdentifier(
                "skeleton",
                "raw",
                hardwareMap.appContext.getPackageName()
        );

        if (soundID != 0) {
            soundFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Sound resource", soundFound ? "Found" : "Not found");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            if (!soundPlaying && soundFound && gamepad1.x) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);

                soundPlaying = false;

                telemetry.addData("Playing", "Sound Resource");
                telemetry.update();
            }
        }
    }
}
