package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Mecanum drivetrain subsystem */
public class Drive extends Subsystem {
    //DC Motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    //Sensors
    private BNO055IMU imu;

    public Drive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx rearLeft, DcMotorEx rearRight, BNO055IMU imu, ElapsedTime timer) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.imu = imu;
        this.timer = timer;
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Stops all drive motors */
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    /** Sets all drive motors to specified run mode */
    public void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        rearLeft.setMode(mode);
        rearRight.setMode(mode);
    }

    /** Sets all drive motors to specified zero power behavior */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
        frontLeft.setZeroPowerBehavior(mode);
        frontRight.setZeroPowerBehavior(mode);
        rearLeft.setZeroPowerBehavior(mode);
        rearRight.setZeroPowerBehavior(mode);
    }

    public void turn(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        rearLeft.setPower(power);
        rearRight.setPower(-power);
    }

    public void setPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    public void setTargetPosition(int targetPosition) {
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(targetPosition);
        rearLeft.setTargetPosition(targetPosition);
        rearRight.setTargetPosition(targetPosition);
    }

    public double getYaw() {
        return imu.getAngularOrientation().firstAngle;
    }
}
