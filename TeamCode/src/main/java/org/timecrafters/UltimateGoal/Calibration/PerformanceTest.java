package org.timecrafters.UltimateGoal.Calibration;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

import java.util.ArrayList;

public class PerformanceTest extends CyberarmState {

    private Robot robot;
    private ArrayList<Float> angles = new ArrayList<Float>();


    public PerformanceTest(Robot robot) {
        this.robot = robot;
    }


    @Override
    public void exec() {
        robot.updateLocation();
//        robot.record();

        if (engine.gamepad1.x) {
            double[] powers = robot.getMecanumPowers(0, 1, 0);
            robot.setDrivePower(powers[0], powers[1], powers[2], powers[3]);
        } else {
            robot.setDrivePower(0,0,0,0);
        }
    }

}
