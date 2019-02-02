package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

@Autonomous(name = "AutoBlueCrater")
public class Auto_Blue_Crater extends LinearOpMode {
    private static final String TAG = "AutoBlueCrater";

    private Robot robot;
    private ElapsedTime timer;

    private VuforiaLocalizer vuforia;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private TFObjectDetector tfod;

    private static final double     DRIVE_SPEED             = 0.4;
    private static final double     TURN_SPEED              = 0.3;


    //Timing Constants
    private static final int PICTOGRAPH_TIMEOUT = 5000;

    //Encoder Constants

    /**
     * If you are standing in the Red Alliance Station looking towards the center of the field,
     *     - The X axis runs from your left to the right. (positive from the center to the right)
     *     - The Y axis runs from the Red Alliance Station towards the other side of the field
     *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
     *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
     **/
    // Field parameters
    private static final double     FIELD_X    = 72.0;
    private static final double     FIELD_Y    = 72.0;
    private static final double     DEPOT_X    = -FIELD_X + 22.0;
    private static final double     DEPOT_Y    = FIELD_Y - 22.0;
    private static final double     CRATER_X    = FIELD_X - 25.0;
    private static final double     CRATER_Y    = FIELD_Y - 25.0;

    // mineral position (left lander) (Crater side)
    private static final double     MINERAL1_X    = FIELD_X - 45.5;
    private static final double     MINERAL1_Y    = FIELD_Y - 25.0;
    private static final double     MINERAL2_X    = FIELD_X - 35.25;
    private static final double     MINERAL2_Y    = FIELD_Y - 35.25;
    private static final double     MINERAL3_X    = FIELD_X - 25.0;
    private static final double     MINERAL3_Y    = FIELD_Y - 45.5;

//    // mineral position (right lander) (Depot side)
//    private static final double     MINERAL1_X    = -FIELD_X + 25.0;
//    private static final double     MINERAL1_Y    = FIELD_Y - 45.5;
//    private static final double     MINERAL2_X    = -FIELD_X + 35.25;
//    private static final double     MINERAL2_Y    = FIELD_Y - 35.25;
//    private static final double     MINERAL3_X    = -FIELD_X + 45.5;
//    private static final double     MINERAL3_Y    = FIELD_Y - 25.0;

    // Robot initial position (left lander) (Crater side)
    private static final double     ROBOT_INIT_POS_X    = 15.0;
    private static final double     ROBOT_INIT_POS_Y    = 15.0;
    private static final double     ROBOT_INIT_ANGLE    = 45.0;

//    // Robot initial position (right lander) (Depot side)
//    private static final double     ROBOT_INIT_POS_X    = -15.0;
//    private static final double     ROBOT_INIT_POS_Y    = 15.0;
//    private static final double     ROBOT_INIT_ANGLE    = 135.0;

    private static final double     ROBOT_HALF_LENGTH    = 9.0;

    private static final int        MAX_TRIAL_COUNT = 6;

    // define robot position global variables
    private double robotCurrentPosX;    // unit in inches
    private double robotCurrentPosY;    // unit in inches
    private double robotCurrentAngle;   // unit in degrees

    // field coordinate polarity
    // no coordinate inversion needed for Blue Alliance
    // coordinate inversion needed for Red Alliance
    private boolean allianceRed = false;

    //Define Vuforia Nav
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private VuforiaTrackables targetsRoverRuckus;
    private List<VuforiaTrackable> allTrackables;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        double startTime = 0.0;
        robot = new Robot(this, timer);

        initRobot();
        waitForStart();

        log("Started Mark 1 Auto");

        //LANDING NEED TO BE WRITTEN

        // remember to engage the slider motor and intake ramp servo so that they do not move during robot motion




        // define robot position after landing
        // should be at (15.0, 15.0)
        robotCurrentPosX = ROBOT_INIT_POS_X;
        robotCurrentPosY = ROBOT_INIT_POS_Y;
        robotCurrentAngle = ROBOT_INIT_ANGLE;
//        calibrateRobotPos();

        // find mineral configuration
        int samplePos;
        int trialCount = 0;
        if (tfod != null) {
            tfod.activate();
        }
        do {
            samplePos = mineralPosIdentification2Mineral_crater();
            trialCount++;
            Thread.sleep(500);
            // repeat MAX_TRIAL_COUNT times if no mineral was found
        } while (trialCount < MAX_TRIAL_COUNT);

        //Forwards towards mineral samples
        moveForward(15.0);      // should be at (25.61, 25.61)
//        moveToPosABS(-25.61, 25.61);    // depot case
//        moveToPosABS( 25.61, 25.61);    // crater case

        //Drive to correct sample (gold)
        switch (samplePos) {
            case 0:         // left
                moveLeft(16.0);     // should be at (14.30, 36.92)
//                moveToPosABS( 14.30, 36.92);    // crater case
                break;
            case 1:         // center
                break;
            case 2:         // right
                moveRight(16.0);    // should be at (36.92, 14.30)
//                moveToPosABS( 36.92, 14.30);    // crater case
                break;
            default: // gold mineral not found, go straight
                break;
        }
        // drive forward to move the mineral
        moveForward(10.0);

        // drive back away from the crater
        moveBackward(10.0);

        //prepare to move to depot depend on sample mineral position
        switch (samplePos) {
            case 0:         // left
                // should be at (14.30, 36.92)
                break;
            case 1:         // center
                moveLeft(16.0);         // should be at (14.30, 36.92)
                break;
            case 2:         // right
                moveLeft(32.0);         // should be at (14.30, 36.92)
                break;
            default:
                moveLeft(16.0);         // should be at (14.30, 36.92)
        }
//        moveToPosABS( 14.30, 36.92);    // crater case

        // move to depot
        turnRobot(90.0);                // should be at (14.30, 36.92)
//        calibrateRobotPos();
//        turnRobot(135.0 - robotCurrentAngle);                // calibrate robot orientation

        moveForward(15.0);              // should be at (3.69, 47.53)
//        moveToPosABS( 3.69, 47.53);    // crater case
        turnRobot(45.0);
        moveRight(13.47);               // should be at (3.69, 61.00)
//        moveToPosABS( 3.69, 61.00);    // crater case
        moveForward(50.0);              // should be at (-46.31, 61.00)
//        moveToPosABS( -46.31, 61.00);    // crater case


        //DROP TEAM MARKER NEED TO BE ADDED





        // Go to crater

        moveBackward(50.0);             // should be at (3.69, 61.00)
//        moveToPosABS( 3.69, 61.00);    // crater case
        moveLeft(6.0);                  // should be at (3.69, 55.00)
//        moveToPosABS( 3.69, 55.00);    // crater case
        turnRobot(180.0);
        moveLeft(6.0);                  // should be at (3.69, 61.00)
//        moveToPosABS( 3.69, 61.00);    // crater case
        moveForward(31.31);             // should be at (35.00, 61.00)
//        moveToPosABS( 35.00, 61.00);    // crater case


        // At the edge of crater

        // deploy intake platform






        if (tfod != null) {
            tfod.shutdown();
        }

    }






    private void initRobot() {
        robot.init();
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Vuforia Init
        initVuforiaEngine();
        telemetry.addLine("Finished Vuforia Initialization.");
        telemetry.update();

        telemetry.addLine("Finished Initialization. Waiting for start.");
        telemetry.update();
        Log.d(TAG, "Finished Initialization. Waiting for start.");
    }

    private void initVuforiaEngine() {
        //Vuforia initialization

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AVjXPzj/////AAABmQ0V3DHJw0P5lI39lVnXqNN+qX1uwVniSS5pN2zeI7ng4z9OkAMad+79Zv+vPtirvt1/Ai6dD+bZL04LynwBqdGmNSXaTXzHd21vpZdiBxmGt9Gb6nMP/p2gTc5wU6hVRJqTe+KexOqzppYs79i5rGbbwO7bZUxpXR5tJeLzicXi3prSnh49SK+kxyTX9XfsjG90+H2TfzVjpYhbX26Qi/abV4uMn7xgzC1q7L54Caixa1aytY3F/NnWAC+87mG5ghf4tcH0CPVFoYEUa0wKMG1bMWOPSfyRG/BBWdaxd1bsIU0xhI5i24nr5LXIrw2JI286TduItR/IH4WRonVA6tbz9QuuhSLlDocIgbwxIbJB";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        telemetry.addLine("Vuforia Init Done");
        telemetry.update();
        //Tensorflow init
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        //Vuforia Navigation Init
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
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
                        }
                        else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        }
                        else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            pos = 0;
                        }
                        else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            pos = 2;
                        }
                        else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            pos = 1;
                        }
                    }
                }
                telemetry.update();
            }
        }

        return pos;
    }

    private int mineralPosIdentification2Mineral_crater(){
        // 0: LEFT
        // 1: CENTER
        // 2: RIGHT
        int pos = -1;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() >= 2) {
                    // there could be many detected objects in the crater
                    // find the two objects closest to the bottom of the image

                    // first define two objects that we want to track
                    int object1X = -1;      // out of bound
                    int object1Y = -1;      // out of bound
                    int object1Type = 0;    // undetermined: 0, gold: 1, silver: 2
                    int object2X = -1;      // out of bound
                    int object2Y = -1;      // out of bound
                    int object2Type = 0;    // undetermined: 0, gold: 1, silver: 2

                    // look for the two objects closest to the bottom of the image
                    for (Recognition recognition : updatedRecognitions) {
                        if (((int) recognition.getBottom()) > object1Y) {
                            // found an object lower than the lowest object
                            // move lowest to second lowest
                            object2X = object1X;
                            object2Y = object1Y;
                            object2Type = object1Type;
                            // update lowest object info
                            object1X = (int) recognition.getLeft();
                            object1Y = (int) recognition.getBottom();
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                object1Type = 1;    // gold
                            }
                            else {
                                object1Type = 2;    // silver
                            }
                        }
                        else if (((int) recognition.getBottom()) > object2Y) {
                            // found an object lower than the second lowest object
                            // update second lowest object info
                            object2X = (int) recognition.getLeft();
                            object2Y = (int) recognition.getBottom();
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                object2Type = 1;    // gold
                            }
                            else {
                                object2Type = 2;    // silver
                            }
                        }
                    }

                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;

                    // check the two minerals closest to tbe bottom of the image
                    if (object1Type == 1) {
                        // object1 is gold
                        goldMineralX = object1X;
                        // object2 is silver
                        silverMineral1X = object2X;
                    }
                    else if (object2Type == 1){
                        // object2 is gold
                        goldMineralX = object2X;
                        // object1 is silver
                        silverMineral1X = object1X;
                    }
                    else {
                        // object1 is silver
                        silverMineral1X = object1X;
                        // object2 is silver
                        silverMineral2X = object2X;
                    }
                    if(goldMineralX == -1){
                        telemetry.addData("Gold Mineral Position", "Left");
                        pos = 0;
                    }
                    else if(goldMineralX < silverMineral1X){
                        telemetry.addData("Gold Mineral Position", "Center");
                        pos = 1;
                    }
                    else {
                        telemetry.addData("Gold Mineral Position", "Right");
                        pos = 2;
                    }
                }
                telemetry.update();
            }
        }

        return pos;
    }

    private double[] robotPosNav() {
        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();

        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.update();
            return new double[]{translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch, rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle};
        }
        else {
            telemetry.addData("Visible Target", "none");
            telemetry.update();
            return null;
        }
    }

    private void calibrateRobotPos() {
        double[] robotPosCalibrationMTRX = robotPosNav();

        if (robotPosCalibrationMTRX != null) {
            // vision target found
            telemetry.addData("old position", "{X, Y, theta} = %.2f, %.2f, %.1f",
                    robotCurrentPosX, robotCurrentPosY, robotCurrentAngle);
            if (allianceRed) {
                // invert coordinate for Red Alliance
                robotCurrentPosX = - robotPosCalibrationMTRX[0];
                robotCurrentPosY = - robotPosCalibrationMTRX[1];
                robotCurrentAngle = robotPosCalibrationMTRX[5] + 180.0;
                if (robotCurrentAngle >= 360.0) robotCurrentAngle -= 360.0;
            }
            else {
                // no need to invert coordinate for Blue Alliance
                robotCurrentPosX = robotPosCalibrationMTRX[0];
                robotCurrentPosY = robotPosCalibrationMTRX[1];
                robotCurrentAngle = robotPosCalibrationMTRX[5];
            }
            telemetry.addData("new position", "{X, Y, theta} = %.2f, %.2f, %.1f",
                    robotCurrentPosX, robotCurrentPosY, robotCurrentAngle);
            telemetry.update();
        }
        else {
            // vision targets were not found
            // no change to robot position information
            telemetry.addData("Visible Target", "not found");
            telemetry.update();
        }
    }

    private void turnRobot(double degrees) {
        robot.drive.turnByAngle(TURN_SPEED, degrees);
//        robotCurrentPosX += ROBOT_HALF_LENGTH * (Math.cos((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.cos(robotCurrentAngle*Math.PI/180.0));
//        robotCurrentPosY += ROBOT_HALF_LENGTH * (Math.sin((robotCurrentAngle+degrees)*Math.PI/180.0)
//                - Math.sin(robotCurrentAngle*Math.PI/180.0));
        robotCurrentAngle += degrees;
        // Display it for the driver.
        telemetry.addData("turnRobot",  "turn to %7.2f degrees", robotCurrentAngle);
        telemetry.update();
        sleep(100);
    }

    private void moveToPosABS(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in absolute field coordinate
        double  deltaX = targetPositionX - robotCurrentPosX;    // in absolute field coordinate
        double  deltaY = targetPositionY - robotCurrentPosY;    // in absolute field coordinate
        double  distanceCountX, distanceCountY;  // distance in motor count in robot coordinate
        // rotate vector from field coordinate to robot coordinate
        distanceCountX = deltaX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0)
                + deltaY * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        distanceCountY = deltaX * Math.cos(robotCurrentAngle*Math.PI/180.0)
                + deltaY * Math.sin(robotCurrentAngle*Math.PI/180.0);
        robot.drive.moveToPos2D(DRIVE_SPEED, distanceCountX, distanceCountY);
        robotCurrentPosX = targetPositionX;
        robotCurrentPosY = targetPositionY;
        // Display it for the driver.
        telemetry.addData("moveToPosABS",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveToPosREL(double targetPositionX, double targetPositionY) {
        // move to (targetPositionX, targetPositionY) in relative robot coordinate
        robot.drive.moveToPos2D(DRIVE_SPEED, targetPositionX, targetPositionY);
        robotCurrentPosX += targetPositionY * Math.cos(robotCurrentAngle*Math.PI/180.0)
                + targetPositionX * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += targetPositionY * Math.sin(robotCurrentAngle*Math.PI/180.0)
                + targetPositionX * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveToPosREL",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveForward(double distance) {
        robot.drive.moveToPos2D(DRIVE_SPEED, 0.0, distance);
        robotCurrentPosX += distance * Math.cos(robotCurrentAngle*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin(robotCurrentAngle*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveForward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveBackward(double distance) {
        robot.drive.moveToPos2D(DRIVE_SPEED, 0.0, -distance);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+180.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+180.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveBackward",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveLeft(double distance) {
        robot.drive.moveToPos2D(DRIVE_SPEED, -distance, 0.0);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle+90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle+90.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveLeft",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

    private void moveRight(double distance) {
        robot.drive.moveToPos2D(DRIVE_SPEED, distance, 0.0);
        robotCurrentPosX += distance * Math.cos((robotCurrentAngle-90.0)*Math.PI/180.0);
        robotCurrentPosY += distance * Math.sin((robotCurrentAngle-90.0)*Math.PI/180.0);
        // Display it for the driver.
        telemetry.addData("moveRight",  "move to %7.2f, %7.2f", robotCurrentPosX,  robotCurrentPosY);
        telemetry.update();
        sleep(100);
    }

}