package org.firstinspires.ftc.teamcode.Auto;
import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

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

    // Field parameters
    private static final double     FIELD_X    = 144.0;
    private static final double     FIELD_Y    = 144.0;
    private static final double     DEPOT_X    = FIELD_X - 22.0;
    private static final double     DEPOT_Y    = 22.0;
    private static final double     CRATER_X    = 25.0;
    private static final double     CRATER_Y    = 25.0;

/*
    // mineral position (left lander)
    private static final double     MINERAL1_X    = 45.5;
    private static final double     MINERAL1_Y    = 25.0;
    private static final double     MINERAL2_X    = 35.25;
    private static final double     MINERAL2_Y    = 35.25;
    private static final double     MINERAL3_X    = 25.0;
    private static final double     MINERAL3_Y    = 45.5;
*/

    // mineral position (right lander)
    private static final double     MINERAL1_X    = FIELD_X - 25.0;
    private static final double     MINERAL1_Y    = 45.5;
    private static final double     MINERAL2_X    = FIELD_X - 35.25;
    private static final double     MINERAL2_Y    = 35.25;
    private static final double     MINERAL3_X    = FIELD_X - 45.5;
    private static final double     MINERAL3_Y    = 25.0;

/*
    // Robot initial position (left lander)
    private static final double     ROBOT_INIT_POS_X    = FIELD_X*0.5 - 21.0;;
    private static final double     ROBOT_INIT_POS_Y    = FIELD_X*0.5 - 21.0;
    private static final double     ROBOT_INIT_ANGLE    = 225.0;
*/

    // Robot initial position (right lander)
    private static final double     ROBOT_INIT_POS_X    = FIELD_X*0.5 + 21.0;;
    private static final double     ROBOT_INIT_POS_Y    = FIELD_X*0.5 - 21.0;
    private static final double     ROBOT_INIT_ANGLE    = 315.0;

    private static final double     ROBOT_HALF_LENGTH    = 9.0;

    private static final int        MAX_TRIAL_COUNT = 3;

    // define robot position global variables
    double robotCurrentPosX;    // unit in inches
    double robotCurrentPosY;    // unit in inches
    double robotCurrentAngle;   // unit in degrees

    //Define Vuforia Nav
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        double startTime = 0.0;
        robot = new Robot(this, timer);

        initRobot();
        waitForStart();

        log("Started Mark 1 Auto");

        //LANDING NEED TO BE WRITTEN

        // define robot position after landing
        robotCurrentPosX = ROBOT_INIT_POS_X;
        robotCurrentPosY = ROBOT_INIT_POS_Y;
        robotCurrentAngle = ROBOT_INIT_ANGLE;

        // find mineral configuration
        int samplePos;
        int trialCount = 0;
        do {
            samplePos = mineralPosIdentification();
            trialCount++;
            // repeat MAX_TRIAL_COUNT times if no mineral was found
        } while ((samplePos == -1) && (trialCount < MAX_TRIAL_COUNT));

        //Forwards towards mineral samples
        moveForward(10.0);

        //Drive to correct sample (gold)
        switch (samplePos) {
            case 0:
                moveLeft(16.0);
                break;
            case 1:
                break;
            case 2:
                moveRight(16.0);
                break;
            default: // gold mineral not found, go straight
                break;
        }
        // drive forward to move the mineral
        moveForward(10.0);

        //move to depot depend on sample mineral position
        switch (samplePos) {
            case 0:
                moveForward(10.0);
                turnRobot(-30.0);
                moveForward(15.0);
                break;
            case 1:
                moveForward(20.0);
                break;
            case 2:
                moveForward(10.0);
                turnRobot(30.0);
                moveForward(15.0);
                break;
            default:
                moveForward(20.0);
        }

        //DROP TEAM MARKER NEED TO BE ADDED


    }






    private void initRobot() {
        robot.init();
        robot.jewel.retract();
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Vuforia initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AVjXPzj/////AAABmQ0V3DHJw0P5lI39lVnXqNN+qX1uwVniSS5pN2zeI7ng4z9OkAMad+79Zv+vPtirvt1/Ai6dD+bZL04LynwBqdGmNSXaTXzHd21vpZdiBxmGt9Gb6nMP/p2gTc5wU6hVRJqTe+KexOqzppYs79i5rGbbwO7bZUxpXR5tJeLzicXi3prSnh49SK+kxyTX9XfsjG90+H2TfzVjpYhbX26Qi/abV4uMn7xgzC1q7L54Caixa1aytY3F/NnWAC+87mG5ghf4tcH0CPVFoYEUa0wKMG1bMWOPSfyRG/BBWdaxd1bsIU0xhI5i24nr5LXIrw2JI286TduItR/IH4WRonVA6tbz9QuuhSLlDocIgbwxIbJB";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

        //Tensorflow init
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        //Vuforia Navigation Init
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

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
//      loop only once
//        while (opModeIsActive()) {
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
//        }
        if (tfod != null) {
            tfod.shutdown();
        }
        return pos;
    }

    private void turnRobot(double degrees) {
        robot.drive.turnByAngle(TURN_SPEED, degrees);
        robotCurrentPosX += ROBOT_HALF_LENGTH * (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
                - Math.cos(robotCurrentAngle*Math.PI/180.0));
        robotCurrentPosY += ROBOT_HALF_LENGTH * (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += degrees;
        // Display it for the driver.
        telemetry.addData("turnRobot",  "turn to %7.2lf degrees", robotCurrentAngle);
        telemetry.update();
        sleep(100);
    }

    private void moveToPosABS(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in absolute field coordinate
        double  deltaX = targetPositionX - robotCurrentPosX;    // in absolute field coordinate
        double  deltaY = targetPositionY - robotCurrentPosY;    // in absolute field coordinate
        double  distanceCountX, distanceCountY;  // distance in motor count in robot coordinate
        // rotate vector from field coordinate to robot coordinate
        distanceCountX = (deltaX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0)
                + deltaY * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0)) * COUNTS_PER_INCH;
        distanceCountY = (deltaX * Math.cos(robotCurrentAngle*Math.PI/180.0)
                + deltaY * Math.sin(robotCurrentAngle*Math.PI/180.0)) * COUNTS_PER_INCH;
        robot.drive.moveToPos2D(DRIVE_SPEED, distanceCountX, distanceCountY);
        robotCurrentPosX = targetPositionX;
        robotCurrentPosY = targetPositionY;
        // Display it for the driver.
        telemetry.addData("moveForward",  "move to %7.2lf, %7.2lf", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveToPosREL(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in relative robot coordinate
        double  distanceCountX, distanceCountY;  // distance in motor count
        distanceCountX = targetPositionX * COUNTS_PER_INCH;
        distanceCountY = targetPositionY * COUNTS_PER_INCH;
        robot.drive.moveToPos2D(DRIVE_SPEED, distanceCountX, distanceCountY);
        robotCurrentPosX += targetPositionY * Math.cos(robotCurrentAngle*Math.PI/180.0)
                        + targetPositionX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += targetPositionY * Math.sin(robotCurrentAngle*Math.PI/180.0)
                        + targetPositionX * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveForward",  "move to %7.2lf, %7.2lf", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveForward(double distance) {
        double  distanceCount;  // distance in motor count
        distanceCount = distance * COUNTS_PER_INCH;
        robot.drive.moveToPos2D(DRIVE_SPEED, 0.0, distanceCount);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveForward",  "move to %7.2lf, %7.2lf", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveBackward(double distance) {
        double  distanceCount;  // distance in motor count
        distanceCount = distance * COUNTS_PER_INCH;
        robot.drive.moveToPos2D(DRIVE_SPEED, 0.0, -distanceCount);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+180.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+180.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveBackward",  "move to %7.2lf, %7.2lf", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveLeft(double distance) {
        double  distanceCount;  // distance in motor count
        distanceCount = distance * COUNTS_PER_INCH;
        robot.drive.moveToPos2D(DRIVE_SPEED, -distanceCount, 0.0);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+90.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveLeft",  "move to %7.2lf, %7.2lf", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveRight(double distance) {
        double  distanceCount;  // distance in motor count
        distanceCount = distance * COUNTS_PER_INCH;
        robot.drive.moveToPos2D(DRIVE_SPEED, distanceCount, 0.0);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveRight",  "move to %7.2lf, %7.2lf", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

}