package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.subsystems.RelicRecovery.RelicState.STOPPED;

/**
 * Created by Cat on 11/10/17.
 */

public class RelicRecovery {

    public Servo wrist;
    public Servo claw;
    public DcMotor relicMotor;
    public ElapsedTime OpModeTime;

    public enum RelicState {
        STOPPED, GRABBED, LIFTING, LIFTED;
    }

    public RelicState relicState = STOPPED;

    public RelicRecovery(Servo wrist, Servo claw, DcMotor relicMotor, ElapsedTime OpModeTime) {
        this.wrist = wrist;
        this.claw = claw;
        this.relicMotor = relicMotor;
        this.OpModeTime = OpModeTime;
    }

    // TO DO: specific functions + positions
    public RelicState getRelicState() {
        return relicState;
    }

    public void setRelicState(RelicState state) {
        relicState = state;
    }

    public void rotateWrist() {
        wrist.setPosition(1); // need to test, not sure what position
    }

    public void deployClaw() {
        claw.setPosition(1);
    }

}