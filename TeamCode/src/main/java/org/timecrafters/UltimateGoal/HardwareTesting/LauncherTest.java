package org.timecrafters.UltimateGoal.HardwareTesting;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class LauncherTest extends CyberarmState {

    DcMotor LaunchMotor;

    @Override
    public void init() {
        LaunchMotor = engine.hardwareMap.dcMotor.get("launcher");
    }

    @Override
    public void exec() {
        LaunchMotor.setPower(1);
    }

    @Override
    public void telemetry() {


    }
}
