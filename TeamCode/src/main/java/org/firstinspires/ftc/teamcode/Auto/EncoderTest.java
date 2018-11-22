package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by tarunsingh on 12/5/17.
 */

@Autonomous (name="Encoder Test")
public class EncoderTest extends LinearOpMode {
    private static final int targetPosition = 315;
    private static final double maxPower = 0.25;

    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();
        Robot robot = new Robot(this, timer);
        robot.init();
        waitForStart();
        timer.reset();
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive.frontLeft.setTargetPosition(targetPosition);
        robot.drive.frontRight.setTargetPosition(targetPosition);
        robot.drive.rearLeft.setTargetPosition(targetPosition);
        robot.drive.rearRight.setTargetPosition(targetPosition);
        robot.drive.setPower(maxPower);
        while (opModeIsActive() && robot.drive.frontLeft.isBusy() && robot.drive.frontRight.isBusy()) {
            telemetry.addData("Front Left", robot.drive.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", robot.drive.frontRight.getCurrentPosition());
            telemetry.addData("Rear Left", robot.drive.rearLeft.getCurrentPosition());
            telemetry.addData("Rear Right", robot.drive.rearRight.getCurrentPosition());
            telemetry.update();
        }
        sleep(1000);
        robot.drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && robot.drive.getYaw() > -86) {
            telemetry.addData("Yaw", robot.drive.getYaw());
            telemetry.update();
            robot.drive.turn(0.25);
        }
        sleep(1000);

        robot.drive.stop();
        while (opModeIsActive()) {}
    }
}
