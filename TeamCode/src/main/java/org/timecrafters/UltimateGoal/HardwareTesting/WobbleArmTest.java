package org.timecrafters.UltimateGoal.HardwareTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.cyberarm.engine.V2.CyberarmState;


public class WobbleArmTest extends CyberarmState {

    private int currentPos;
    private DcMotor motor;

    @Override
    public void init() {
        motor = engine.hardwareMap.dcMotor.get("wobbleArm");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void exec() {
        currentPos = motor.getCurrentPosition();
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Position", currentPos);
    }
}
