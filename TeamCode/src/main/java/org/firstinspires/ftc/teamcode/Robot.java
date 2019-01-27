package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
/**
 * Created by AndrewC on 11/25/2017.
 */

public class Robot {
    public String name;
    private HardwareMap hardwareMap;
    private ElapsedTime timer;

    //DC Motors
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx rearRightDriveMotor;
    private DcMotorEx rearLeftDriveMotor;
    private DcMotorEx leftGlyftMotor;
    private DcMotorEx rightGlyftMotor;
    private DcMotorEx relicMotor;

    //Servos
//    private Servo jewelServo;
//    private Servo leftSqueezerServo;
//    private Servo rightSqueezerServo;
//    private Servo relicWristServo;
//    private Servo relicClawServo;
    private Servo intakeBoxTilt;
    private Servo intakeMech;


    //Sensors
    private BNO055IMU imu;
    private ColorSensor colorSensor;

    //Subsystems
    public Drive drive;

    public Robot(OpMode opMode, ElapsedTime timer){
        hardwareMap = opMode.hardwareMap;
        this.timer = timer;
        init();
    }
    public void init(){
        //DC Motors
        frontLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        frontRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        rearLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        rearRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("br");

        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos
//        jewelServo = hardwareMap.servo.get("jewel_servo");
//        leftSqueezerServo = hardwareMap.servo.get("left_grabber");
//        rightSqueezerServo = hardwareMap.servo.get("right_grabber");
//        relicWristServo = hardwareMap.servo.get("relic_wrist");
//        relicClawServo = hardwareMap.servo.get("relic_claw");

        //Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        colorSensor = hardwareMap.colorSensor.get("color");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        //Subsystems
        drive = new Drive(frontLeftDriveMotor, frontRightDriveMotor, rearLeftDriveMotor, rearRightDriveMotor, imu, timer);
//        glyft = new Glyft(leftSqueezerServo, rightSqueezerServo, leftGlyftMotor, rightGlyftMotor, timer);
//        relicRecovery = new RelicRecovery(relicWristServo, relicClawServo, relicMotor, timer);
//        jewel = new Jewel(jewelServo, colorSensor, timer);
    }
}

