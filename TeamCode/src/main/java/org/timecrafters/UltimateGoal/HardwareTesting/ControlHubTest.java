package org.timecrafters.UltimateGoal.HardwareTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class ControlHubTest extends CyberarmState {

    private Robot robot;
    private float angle = 0;

    public ControlHubTest(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        angle = robot.imu.getAngularOrientation().firstAngle;
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("Greetings");
        engine.telemetry.addData("Angle", angle);

    }
}
