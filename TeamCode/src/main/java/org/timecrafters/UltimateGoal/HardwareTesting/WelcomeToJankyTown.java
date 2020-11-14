package org.timecrafters.UltimateGoal.HardwareTesting;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;


public class WelcomeToJankyTown extends CyberarmState {

    DcMotor left;
    DcMotor right;

    @Override
    public void init() {
        left = engine.hardwareMap.dcMotor.get("left");
        right = engine.hardwareMap.dcMotor.get("right");
    }

    @Override
    public void exec() {
        left.setPower(-engine.gamepad1.left_stick_y);
        right.setPower(engine.gamepad1.right_stick_y);
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("Welcome to Janky Town!");

    }
}
