package org.timecrafters.UltimateGoal.HardwareTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;


public class FullTest extends CyberarmState {

    private Robot robot;

    public FullTest(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {

    }

    @Override
    public void exec() {
        double p = 0;

        if (engine.gamepad1.x) {
            p = 0.1;
        }


        robot.setDrivePower(p,p,p,p);
        robot.collectionMotor.setPower(0.5);
        robot.ringBeltMotor.setPower(p);
        robot.launchMotor.setPower(p);
    }

    @Override
    public void telemetry() {

    }
}
