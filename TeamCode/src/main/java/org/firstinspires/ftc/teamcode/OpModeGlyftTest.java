package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by tarunsingh on 11/18/17.
 */

@TeleOp(name = "Glyft Test")
public class OpModeGlyftTest extends LinearOpMode {
    private DcMotorEx lift1;
    private DcMotorEx lift2;

    private double motorPower = 1.00;

    @Override
    public void runOpMode() {
        lift1 = (DcMotorEx) hardwareMap.dcMotor.get("lift1");
        lift2 = (DcMotorEx) hardwareMap.dcMotor.get("lift2");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motorPower = 0.10;
            }
            if (gamepad1.b) {
                motorPower = 0.20;
            }
            if (gamepad1.x) {
                motorPower = 0.30;
            }
            if (gamepad1.y) {
                motorPower = 0.40;
            } else {
                motorPower = 0.00;
            }

            if (gamepad1.left_bumper) {
                motorPower = -0.25;
            }
            /*
            if (gamepad1.a) {
                lift1.setPower(motorPower);
                lift2.setPower(motorPower);
            }
            if (gamepad1.y) {
                lift1.setPower(-motorPower);
                lift2.setPower(-motorPower);
            } else {
                lift1.setPower(0);
                lift2.setPower(0);
            }
            */
            lift1.setPower(motorPower);
            lift2.setPower(motorPower);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();
        }
    }
}
