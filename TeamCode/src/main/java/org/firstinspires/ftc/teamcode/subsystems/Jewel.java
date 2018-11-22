package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Jewel arm subsystem */
public class Jewel extends Subsystem {
    //Constants
    private static final double DEPLOYED_POSITION = 0.02;
    private static final double STORED_POSITION = 0.5;

    //Servos
    private Servo arm;

    //Sensors
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;

    public Jewel(Servo arm, ColorSensor leftSensor, ElapsedTime timer) {
        this.arm = arm;
        this.leftSensor = leftSensor;
        this.timer = timer;
    }

    /** Deploys jewel arm to reading position */
    public void deploy() {
        arm.setPosition(DEPLOYED_POSITION);
    }

    /** Retracts jewel arm to stored position */
    public void retract() {
        arm.setPosition(STORED_POSITION);
    }

    /** Returns true if RED | BLUE; false if BLUE | RED */
    public boolean detectJewels1(){
        //TODO: Refine detection logic
        int leftRed = leftSensor.red();
        int leftBlue = leftSensor.blue();
        return leftRed > leftBlue;
    }
    public boolean detectJewels2() throws JewelDetectionException {
        //TODO: Refine detection logic
        int leftRed = leftSensor.red();
        int leftBlue = leftSensor.blue();
        return leftRed > leftBlue;
    }

    /** Error thrown when jewel detection fails */
    private class JewelDetectionException extends Exception {

    }
}
