package org.timecrafters.UltimateGoal.HardwareTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class EncoderTest extends CyberarmState {

    private Robot robot;

    public EncoderTest(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {

        robot.setDrivePower(1, 1,1,1);

    }
}
