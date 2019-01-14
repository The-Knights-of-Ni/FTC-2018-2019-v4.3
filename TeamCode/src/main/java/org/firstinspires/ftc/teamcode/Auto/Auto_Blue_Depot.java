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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;

@Autonomous(name = "AutoBlueDepot")
public class Auto_Blue_Depot extends LinearOpMode{
    private static final String TAG = "AutoBlueDepot";

    private Robot robot;
    private ElapsedTime timer;

    private VuforiaLocalizer vuforia;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private TFObjectDetector tfod;

    //DO WITH ENCODERS
    private static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // AM Orbital 20 motor
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;

    //Timing Constants
    private static final int PICTOGRAPH_TIMEOUT = 5000;

    //Encoder Constants

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        double startTime = 0.0;
        robot = new Robot(this, timer);

        initRobot();
        waitForStart();

        log("Started Mark 1 Auto");

        //LANDING NEED TO BE WRITTEN

        int samplePos = mineralPosIdentification();
        //Forwards towards mineral samples
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.setTargetPosition(??);
        robot.drive.setPower(0.10);
        while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy()) {

        }
        robot.drive.stop();
        sleep(500);

        //Drive to correct sample (gold)
        int targetPosition = 0;
        switch (samplePos) {
            case 0:
                targetPosition = ???;
                break;
            case 1:
                targetPosition = ???;
                break;
            case 2:
                targetPosition = ???;
                break;
            default:
                targetPosition = ???;
        }
        moveToPos(0.10, targetPosition);

        moveToPos(0.10, ???);
        moveToPos(0.10,???);

        //move to depot depend on sample mineral position
        switch (samplePos) {
            case 0:
                targetPosition = ???;
                break;
            case 1:
                targetPosition = ???;
                break;
            case 2:
                targetPosition = ???;
                break;
            default:
                targetPosition = ???;
        }
        moveToPos(0.10, targetPosition);

        //DROP TEAM MARKER NEED TO BE ADDED




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

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Tensorflow init
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        telemetry.addLine("Finished Initialization. Waiting for start.");
        telemetry.update();
        Log.d(TAG, "Finished Initialization. Waiting for start.");
    }

    private void log(String message) {
        telemetry.addLine(message);
        telemetry.update();
        Log.d(TAG, message);
    }

    private int mineralPosIdentification(){
        // 0: LEFT
        // 1: CENTER
        // 2: RIGHT
        int pos = -1;
        if (tfod != null) {
            tfod.activate();
        }

        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                pos = 0;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                pos = 1;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                pos = 2;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        return pos;
    }

    public void moveToPos(double power, int targetPosition){
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.setTargetPosition(targetPosition);
        robot.drive.setPower(power);
        while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy()) {

        }
        robot.drive.stop();
        sleep(500);
    }
}