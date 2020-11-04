package org.timecrafters.UltimateGoal.HardwareTesting;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class LauncherTest extends CyberarmState {

    DcMotor LaunchMotor;
    DcMotor SuckMotor;

    @Override
    public void init() {
        LaunchMotor = engine.hardwareMap.dcMotor.get("launcher");
        SuckMotor = engine.hardwareMap.dcMotor.get("collector");
    }

    @Override
    public void exec() {
        LaunchMotor.setPower(1);
        SuckMotor.setPower(1);
    }

    @Override
    public void telemetry() {


    }
}
