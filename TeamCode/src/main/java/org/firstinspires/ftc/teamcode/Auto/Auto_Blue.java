package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by AndrewC on 11/25/2017.
 */
@Autonomous(name = "Auto Blue")
public class Auto_Blue extends LinearOpMode {
    private static final String TAG = "Mark1Auto";

    private Robot robot;
    private ElapsedTime timer;

    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    //DO WITH ENCODERS
    private static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // AM Orbital 20 motor
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;

    //Timing Constants
    private static final int PICTOGRAPH_TIMEOUT = 5000;
    private static final int JEWEL_DEPLOY_WAIT = 1500;
    private static final double GLYFT_LIFT_TIME = 0.25;

    //Encoder Constants

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        double startTime = 0.0;
        robot = new Robot(this, timer);

        initRobot();
        waitForStart();

        log("Started Mark 1 Auto");

        //Grab glyft and raise lift
        robot.glyft.closeSqueezers();
        sleep(500);
        startTime = timer.seconds();
        while (opModeIsActive() && (timer.seconds() - startTime) < GLYFT_LIFT_TIME) {
            robot.glyft.setPower(0.50);
        }
        robot.glyft.setPower(0);
        sleep(500);

        //Deploy jewel arm and read jewel color
        robot.jewel.deploy();
        sleep(JEWEL_DEPLOY_WAIT);
        boolean jewelIsRed = robot.jewel.detectJewels1();
        log("Jewel Detected: " + (jewelIsRed ? "RED | BLUE" : "BLUE | RED"));
        sleep(500);

        //Knock off correct jewel
        relicTrackables.activate();
        sleep(1000);
        startTime = timer.seconds();
        int targetPosition = 0;
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (jewelIsRed) {
            targetPosition = 160;
        } else {
            targetPosition = -900;
        }
        robot.drive.setTargetPosition(targetPosition);
        robot.drive.setPower(0.10);

        RelicRecoveryVuMark vuMark = null;
        while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        robot.drive.stop();
        robot.jewel.retract();
        relicTrackables.deactivate();
        sleep(500);

        //If backwards, drive to pictograph reading position
        if (jewelIsRed) {
            robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.drive.setTargetPosition(-1060);
            robot.drive.setPower(0.10);
            while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy()) {

            }
            robot.drive.stop();
            sleep(500);
        }

        /*
        //Read pictograph
        RelicRecoveryVuMark vuMark = null;
        relicTrackables.activate();
        startTime = timer.seconds();
        while (opModeIsActive() && (timer.seconds() - startTime) < PICTOGRAPH_TIMEOUT) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                break;
            }
        }
        relicTrackables.deactivate();
        log("Finished reading pictograph: " + vuMark);
        */

        //Drive to correct column
        switch (vuMark) {
            case RIGHT:
                targetPosition = -945;
                break;
            case CENTER:
                targetPosition = -630;
                break;
            case LEFT:
                targetPosition = -315;
                break;
            default:
                targetPosition = -630;
        }
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.setTargetPosition(targetPosition);
        robot.drive.setPower(0.10);
        while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy()) {

        }
        robot.drive.stop();
        sleep(500);

        //Turn to face cryptobox
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && Math.abs(robot.drive.getYaw() - (-90)) > 2) {
            robot.drive.turn(0.10);
        }
        robot.drive.stop();
        sleep(500);

        //Drive forward to approach cryptobox
        robot.glyft.openSqueezers();
        sleep(500);
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startTime = timer.seconds();
        while (opModeIsActive() && (timer.seconds() - startTime) < 1.5) {
            robot.drive.setPower(0.10);
        }
        robot.drive.stop();
        sleep(500);

        startTime = timer.seconds();
        while (opModeIsActive() && (timer.seconds() - startTime) < 0.5) {
            robot.drive.setPower(-0.10);
        }
        robot.drive.stop();
        /*
        RelicRecoveryVuMark vuMark = null;
        relicTrackables.activate();
        startTime = timer.seconds();
        while (opModeIsActive() && (timer.seconds() - startTime) < 1.5) {
            robot.drive.setPower(0.10);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                break;
            }
        }
        robot.drive.stop();
        while (opModeIsActive()) {
            telemetry.addData("VuMark", vuMark);
            telemetry.update();
        }
        */

        /*
        timer.reset();

        if (isRed) {
            robot.drive.turn(0.1);
            while (opModeIsActive() && robot.drive.getYaw() >= -10) {
                telemetry.addData("IMU Yaw", robot.drive.getYaw());
                telemetry.update();
            }
            robot.drive.stop();
            robot.jewel.retract();
            sleep(1000);
            robot.drive.turn(-0.1);
            while (opModeIsActive() && robot.drive.getYaw() <= 0) {
                telemetry.addData("IMU Yaw", robot.drive.getYaw());
                telemetry.update();
            }
            robot.drive.stop();
        } else {
            while (opModeIsActive() && timer.seconds() < 1.5) {
                robot.drive.setPower(0.08);
            }
            robot.drive.stop();
        }

        while (opModeIsActive()) {

        }

        /*
        while (opModeIsActive()) {
            if (isRed) {
                robot.drive.turn(0.1);
            } else {
                robot.drive.setPower(0.08);
            }
        }
        */
        /*
        while(opModeIsActive()){
            int countForColor = 0;
            boolean isRed = robot.jewel.detectJewels1();
            sleep(500);
            telemetry.addData("Color", isRed);
            telemetry.update();

            if (!isRed)
            {
                telemetry.addData("detected Blue", isRed);
                telemetry.update();
                driveBackward(1000);
                robot.jewel.retract();
                driveForward(1000);
            }

            driveForward(1000);
            robot.jewel.retract();

            //stop moving
            robot.drive.stop();
            break;

            //move forward - using encoders


            //then use PID control to go to pictograph and read it
        }
        */

    }

    public void driveBackward(int timeLen)
    {
        robot.drive.setPower(-0.5);
        sleep(timeLen);
    }

    public void driveForward(int timeLen)
    {
        robot.drive.setPower(0.5);
        sleep(timeLen);
    }

    /*public void driveForwardEncoder(double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS, Robot robot)
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            robot.frontLeft(robot.frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
    }*/

    private double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX)
    {
        //probably a better way to do this
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        double lrPower = r * Math.sin(robotAngle) + rightStickX;
        double lfPower = r * Math.cos(robotAngle) + rightStickX;
        double rrPower = r * Math.cos(robotAngle) - rightStickX;
        double rfPower = r * Math.sin(robotAngle) - rightStickX;
        return new double[]{lrPower, lfPower, rrPower, rfPower};
    }

    private void initRobot() {
        robot.init();
        robot.jewel.retract();
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Vuforia initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AX1T/JH/////AAAAGWQh21MbJEIhpE//dkoSovcwBwhbe6+121U+fGQaCJZI0cDQka2Bqcnc1N9dRlzyr5ZwjGPLUqxXId7+l/yUFBV1v66pF5nuD5JJOr9IVM22ZUxMSQesMrpCqfzGowHAv/dTDZmuqOxfqazZ6xeJ5V/V/2HdwGFDCrTXbZd4PzSwaOQed48I7XtIvu2m3nEJAb+aAC6DT78HHLRIFStmgfS4QglTEy+M7JOtDkc5u5k5CQhk9hwNsea4nDqfVf9XJjKLJJFhTat0IdiPz8BIrsNWxP8S7EiZLaWdanHJIOdP2NhokmI0jkLgPuRLkC7BvorDDeVI+pdutDMjN9kf/b11uGyrf6fJ4AySTe1+R9m/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        telemetry.addLine("Finished Initialization. Waiting for start.");
        telemetry.update();
        Log.d(TAG, "Finished Initialization. Waiting for start.");
    }

    private void log(String message) {
        telemetry.addLine(message);
        telemetry.update();
        Log.d(TAG, message);
    }
}
