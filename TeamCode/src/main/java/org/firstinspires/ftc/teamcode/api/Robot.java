/*
 * Copyright 2020 FIRST Tech Challenge Team 11792
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.api;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.api.bp.AbstractRobot;
import org.firstinspires.ftc.teamcode.api.bp.DistanceSensorRobot;
import org.firstinspires.ftc.teamcode.api.bp.DuckRobot;
import org.firstinspires.ftc.teamcode.api.bp.GyroRobot;
import org.firstinspires.ftc.teamcode.api.bp.TwoArmRobot;
import org.firstinspires.ftc.teamcode.api.bp.VuforiaRobot;
import org.firstinspires.ftc.teamcode.api.bp.WheeledRobot;
import org.firstinspires.ftc.teamcode.api.config.Constants;
import org.firstinspires.ftc.teamcode.api.config.Naming;
import org.firstinspires.ftc.teamcode.api.hw.Gyroscope;
import org.firstinspires.ftc.teamcode.api.hw.SmartColorSensor;
import org.firstinspires.ftc.teamcode.api.hw.SmartMotor;
import org.firstinspires.ftc.teamcode.api.hw.SmartServo;
import org.firstinspires.ftc.teamcode.api.util.DuckPos;
import org.firstinspires.ftc.teamcode.opmode.teleop.TensorFlowObjectDetectionWebcam;

import java.util.List;
import java.util.concurrent.ExecutorService;

import lombok.Getter;
import lombok.Setter;

public class Robot extends AbstractRobot implements WheeledRobot, DuckRobot, TwoArmRobot, DistanceSensorRobot, GyroRobot, VuforiaRobot {
    public static ExecutorService executorService;
    @Getter
    @Setter
    private static Gyroscope gyro0, gyro1;
    @Getter
    private SmartMotor bl, br, fl, fr, duck;
    @Getter
    private SmartServo armLeft, armRight, clawLeft, clawRight, clawLeft2, clawRight2, armLeft2, armRight2;
    @Getter
    private DistanceSensor distanceFL, distanceFR, distanceBL, distanceBR;
    @Getter
    private SmartColorSensor parkSensor;
    @Getter
    private WebcamName webcam0;
    @Getter
    private LinearOpMode opMode;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    public Robot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void powerWheels(double flPower, double frPower, double blPower, double brPower) {
        this.fl.setPower(flPower);
        this.fr.setPower(frPower);
        this.bl.setPower(blPower);
        this.br.setPower(brPower);
    }

    // Arm 1

    @Override
    public void positionArm(double position) {
        this.armLeft.setPosition(position);
        this.armRight.setPosition(position);
    }

    @Override
    public void positionClaw(double position) {
        this.clawLeft.setPosition(position);
        this.clawRight.setPosition(position);
    }

    public void adjustArm(double amount) {
        this.armLeft.setPosition(this.armLeft.getPosition() + amount);
        this.armRight.setPosition(this.armRight.getPosition() + amount);
    }

    @Override
    public void adjustClaw(double amount) {
        this.clawLeft.setPosition(this.clawLeft.getPosition() + amount);
        this.clawRight.setPosition(this.clawRight.getPosition() + amount);
    }

    // Arm 2

    @Override
    public void positionArm2(double position) {
        this.armLeft2.setPosition(position);
        this.armRight2.setPosition(position);
    }

    @Override
    public void positionClaw2(double position) {
        this.clawLeft2.setPosition(position);
        this.clawRight2.setPosition(position);
    }

    @Override
    public void adjustArm2(double amount) {
        this.armLeft2.setPosition(this.armLeft2.getPosition() + amount);
        this.armRight2.setPosition(this.armRight2.getPosition() + amount);
    }

    @Override
    public void adjustClaw2(double amount) {
        this.clawLeft2.setPosition(this.clawLeft2.getPosition() + amount);
        this.clawRight2.setPosition(this.clawRight2.getPosition() + amount);
    }

    @Override
    public void openClaw() {
        this.positionClaw(0.1);
    }

    @Override
    public void closeClaw() {
        this.positionClaw(0.2);
    }

    @Override
    public void openClaw2() {
        this.positionClaw2(0.1);
    }

    @Override
    public void closeClaw2() {
        this.positionClaw2(0.2);
    }

    @Override
    public void powerDuck(double power) {
        this.duck.setPower(power);
    }

    @Override
    public double getFLDistance() {
        return this.distanceFL.getDistance(DistanceUnit.CM);
    }

    @Override
    public double getFRDistance() {
        return this.distanceFR.getDistance(DistanceUnit.CM);
    }

    @Override
    public double getBLDistance() {
        return this.distanceBL.getDistance(DistanceUnit.CM);
    }

    @Override
    public double getBRDistance() {
        return this.distanceBR.getDistance(DistanceUnit.CM);
    }

    @Override
    public void initTeleOp(@NonNull LinearOpMode opMode) {
        this.opMode = opMode;

        // Motors
        this.bl = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BL));
        this.br = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_BR));
        this.fl = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FL));
        this.fr = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_FR));

        this.duck = new SmartMotor(opMode.hardwareMap.get(DcMotorEx.class, Naming.MOTOR_DUCK));

        // Servos
        this.armLeft = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_LEFT));
        this.armRight = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_RIGHT));
        this.clawLeft = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_LEFT));
        this.clawRight = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_RIGHT));

        this.armLeft2 = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_LEFTo));
        this.armRight2 = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_ARM_RIGHTo));
        this.clawLeft2 = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_LEFTo));
        this.clawRight2 = new SmartServo(opMode.hardwareMap.get(Servo.class, Naming.SERVO_CLAW_RIGHTo));

        // Sensors
        this.distanceFL = opMode.hardwareMap.get(DistanceSensor.class, Naming.SENSOR_DISTANCE_FL);
        this.distanceFR = opMode.hardwareMap.get(DistanceSensor.class, Naming.SENSOR_DISTANCE_FR);
        this.distanceBL = opMode.hardwareMap.get(DistanceSensor.class, Naming.SENSOR_DISTANCE_BL);
        this.distanceBR = opMode.hardwareMap.get(DistanceSensor.class, Naming.SENSOR_DISTANCE_BR);

        // Looped Config
        SmartMotor[] wheelLoop = new SmartMotor[]{this.fl, this.fr, this.bl, this.br};
        SmartServo[] servoLoop = new SmartServo[]{this.armLeft, this.armRight, this.clawLeft, this.clawRight};

        for (SmartMotor motor : wheelLoop) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (SmartServo servo : servoLoop) {
            servo.setPosition(servo.getPosition());
        }

        // Specific Config
        this.fl.setDirection(DcMotor.Direction.REVERSE);
        this.fr.setDirection(DcMotor.Direction.REVERSE);
        this.bl.setDirection(DcMotor.Direction.REVERSE);
        this.br.setDirection(DcMotor.Direction.FORWARD);

        this.armLeft.setDirection(Servo.Direction.FORWARD);
        this.armRight.setDirection(Servo.Direction.REVERSE);
        this.clawLeft.setDirection(Servo.Direction.FORWARD);
        this.clawRight.setDirection(Servo.Direction.REVERSE);

        this.armLeft2.setDirection(Servo.Direction.FORWARD);
        this.armRight2.setDirection(Servo.Direction.REVERSE);
        this.clawLeft2.setDirection(Servo.Direction.FORWARD);
        this.clawRight2.setDirection(Servo.Direction.REVERSE);

        this.bl.setPowerModifier(Constants.MOTOR_BL_POWER_MOD);
        this.br.setPowerModifier(Constants.MOTOR_BR_POWER_MOD);
        this.fl.setPowerModifier(Constants.MOTOR_FL_POWER_MOD);
        this.fr.setPowerModifier(Constants.MOTOR_FR_POWER_MOD);
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newBRTarget;
        int newBLTarget;
        int newFRTarget;
        int newFLTarget;
        int moveCounts;
        double error;
        double steer;
        double lPower;
        double rPower;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * Constants.COUNTS_PER_INCH);
            newBRTarget = br.getCurrentPosition() + moveCounts;
            newBLTarget = bl.getCurrentPosition() + moveCounts;
            newFRTarget = fr.getCurrentPosition() + moveCounts;
            newFLTarget = fl.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            br.setTargetPosition(newBRTarget);
            fr.setTargetPosition(newFRTarget);
            bl.setTargetPosition(newBLTarget);
            fl.setTargetPosition(newFLTarget);

            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            br.setPower(speed);
            fr.setPower(speed);
            bl.setPower(speed);
            fl.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (fl.isBusy() && bl.isBusy() && fr.isBusy() && fl.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, Constants.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) {
                    steer *= -1.0;
                }

                lPower = speed - steer;
                rPower = speed + steer;

                br.setPower(rPower);
                fr.setPower(rPower);
                bl.setPower(lPower);
                fl.setPower(lPower);

                opMode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                opMode.telemetry.addData("FL Target", newFLTarget);
                opMode.telemetry.addData("FR Target", newFRTarget);
                opMode.telemetry.addData("BL Target", newBLTarget);
                opMode.telemetry.addData("BR Target", newBRTarget);
                opMode.telemetry.addData("FL Actual", fl.getCurrentPosition());
                opMode.telemetry.addData("FR Actual", fr.getCurrentPosition());
                opMode.telemetry.addData("BL Actual", bl.getCurrentPosition());
                opMode.telemetry.addData("BR Actual", br.getCurrentPosition());
                opMode.telemetry.addData("FL Speed", fl.getPower());
                opMode.telemetry.addData("FR Speed", fr.getPower());
                opMode.telemetry.addData("BL Speed", bl.getPower());
                opMode.telemetry.addData("BR Speed", br.getPower());
                opMode.telemetry.update();

            }

            // Stop all motion;
            br.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            fl.setPower(0);

            // Turn off RUN_TO_POSITION
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(speed, angle, Constants.P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, Constants.P_TURN_COEFF);
            opMode.telemetry.update();
        }

        // Stop all motion;
        br.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        fl.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return If the robot is on the correct heading
     */
    public boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double lPower;
        double rPower;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= Constants.HEADING_THRESHOLD) {
            steer = 0.0;
            rPower = 0.0;
            lPower = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            lPower = -steer;
            rPower = steer;
        }

        // Send desired speeds to motors.
        br.setPower(rPower);
        fr.setPower(rPower);
        bl.setPower(lPower);
        fl.setPower(lPower);

        // Display it for the driver.
        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("FL Speed", fl.getPower());
        opMode.telemetry.addData("FR Speed", fr.getPower());
        opMode.telemetry.addData("BL Speed", bl.getPower());
        opMode.telemetry.addData("BR Speed", br.getPower());

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- pi. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -pi to +pi range  (
        opMode.telemetry.addData("Current Angle:", gyro0.getAngularOrientation().firstAngle);
        robotError = targetAngle - gyro0.getAngularOrientation().firstAngle;
        while (robotError > Math.PI) robotError -= 2*Math.PI;
        while (robotError < -Math.PI) robotError += 2*Math.PI;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return How much to steer
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = BuildConfig.VUFORIA_KEY;
        parameters.cameraName = webcam0;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

    }

    public void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(Constants.TFOD_MODEL_ASSET, Constants.LABELS);
    }

    public DuckPos getDuckPos() {
        DuckPos pos = DuckPos.NONE;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        if (opMode.opModeIsActive()) {
            // Five seconds ought to do it
            while (opMode.opModeIsActive() && timer.time() < 5) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLeft() < 120) {
                                pos = DuckPos.LEFT;
                            } else if (recognition.getLeft() > 390) {
                                pos = DuckPos.CENTER;
                            } else {
                                pos = DuckPos.RIGHT;
                            }
                            opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            opMode.telemetry.addData("  position: ", pos);
                            i++;
                        }
                        opMode.telemetry.update();
                    }
                }
            }
        }
        return pos;
    }
}
