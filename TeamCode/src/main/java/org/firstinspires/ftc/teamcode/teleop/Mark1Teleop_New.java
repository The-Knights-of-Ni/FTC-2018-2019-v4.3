//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
///**
// * Created by tarunsingh on 12/2/17.
// */
//
//@TeleOp(name = "Mark 1 Teleop")
//public class Mark1Teleop_New extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        ElapsedTime timer = new ElapsedTime();
//        Robot robot = new Robot(this, timer);
//        telemetry.addLine("Initialization finished. Waiting for start...");
//        telemetry.update();
//        waitForStart();
//        boolean lbWasPressedLastLoop = false;
//        boolean rbWasPressedLastLoop = false;
//        boolean upWasPressedLastLoop = false;
//        boolean downWasPressedLastLoop = false;
//        boolean leftWasPressedLastLoop = false;
//        boolean rightWasPressedLastLoop = false;
//        double leftServoPosition = 0.5;
//        double rightServoPosition = 0.5;
//        double clawServoPosition = 0.5;
//
//        double lastIncreaseTime;
//
//        while (opModeIsActive()) {
//            double leftStickX = gamepad1.left_stick_x;
//            double leftStickY = -gamepad1.left_stick_y;
//            double rightStickX = gamepad1.right_stick_x;
//            double[] motorPowers = calcMotorPowers(leftStickX, leftStickY, rightStickX);
//            robot.drive.rearLeft.setPower(motorPowers[0]);
//            robot.drive.frontLeft.setPower(motorPowers[1]);
//            robot.drive.rearRight.setPower(motorPowers[2]);
//            robot.drive.frontRight.setPower(motorPowers[3]);
//
//            //robot.glyft.setPower(-gamepad2.right_stick_y);
//            double motorPower;
//            if (gamepad2.a) {
//                motorPower = 0.30;
//            } else if (gamepad2.b) {
//                motorPower = 0.40;
//            } else if (gamepad2.x) {
//                motorPower = 0.50;
//            } else if (gamepad2.y) {
//                motorPower = 0.60;
//            } else if (gamepad2.right_stick_button) {
//                motorPower = 1.00;
//            } else {
//                motorPower = 0.00;
//            }
//
//            if (gamepad2.left_trigger > 0.5) {
//                motorPower = -0.40;
//            }
//
//            robot.glyft.setPower(motorPower);
//
//            if (gamepad1.left_bumper) { //Open
//                robot.glyft.squeezerLeft.setPosition(0.3);
//                robot.glyft.squeezerRight.setPosition(0.45);
//            } else if (gamepad1.right_bumper) { //Closed
//                robot.glyft.squeezerLeft.setPosition(0.15);
//                robot.glyft.squeezerRight.setPosition(0.55);
//            }
//
//            if (gamepad2.dpad_up) {
//                if (!upWasPressedLastLoop) {
//                    rightServoPosition += 0.05;
//                    upWasPressedLastLoop = true;
//                }
//            } else {
//                upWasPressedLastLoop = false;
//            }
//
//            if (gamepad2.dpad_down) {
//                if (!downWasPressedLastLoop) {
//                    rightServoPosition -= 0.05;
//                    downWasPressedLastLoop = true;
//                }
//            } else {
//                downWasPressedLastLoop = false;
//            }
//
//            if (gamepad2.dpad_left) {
//                robot.relicRecovery.wrist.setPosition(0);
//            } else if (gamepad2.dpad_right) {
//                robot.relicRecovery.wrist.setPosition(0.85);
//            }
//
//            if (gamepad1.dpad_left) {
//                if (!leftWasPressedLastLoop) {
//                    clawServoPosition -= 0.05;
//                    leftWasPressedLastLoop = true;
//                }
//            } else {
//                leftWasPressedLastLoop = false;
//            }
//
//            if (gamepad1.dpad_right) {
//                if (!rightWasPressedLastLoop) {
//                    clawServoPosition += 0.05;
//                    rightWasPressedLastLoop = true;
//                }
//            } else {
//                rightWasPressedLastLoop = false;
//            }
//
//            //robot.glyft.squeezerLeft.setPosition(leftServoPosition);
//            //robot.glyft.squeezerRight.setPosition(rightServoPosition);
//            robot.relicRecovery.claw.setPosition(clawServoPosition);
//            //robot.relicRecovery.wrist.setPosition(rightServoPosition);
//            robot.relicRecovery.relicMotor.setPower(Math.pow(gamepad2.left_stick_y, 3.0));
//
//            telemetry.addData("Servo position L", leftServoPosition);
//            telemetry.addData("Servo position R", rightServoPosition);
//            telemetry.addData("Servo position claw", clawServoPosition);
//            telemetry.update();
//        }
//    }
//
//    private double[] calcMotorPowers(double leftStickX, double leftStickY, double rightStickX) {
//        double r = Math.hypot(leftStickX, leftStickY);
//        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
//        double lrPower = r * Math.sin(robotAngle) + rightStickX;
//        double lfPower = r * Math.cos(robotAngle) + rightStickX;
//        double rrPower = r * Math.cos(robotAngle) - rightStickX;
//        double rfPower = r * Math.sin(robotAngle) - rightStickX;
//        lrPower *= 0.6;
//        lfPower *= 0.6;
//        rrPower *= 0.6;
//        rfPower *= 0.6;
//        return new double[]{lrPower, lfPower, rrPower, rfPower};
//    }
//}
