package org.timecrafters.UltimateGoal.LocalizerTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class VuforiaNavTesting extends CyberarmState {

    private Robot robot;
    private float angle;
    private double X;
    private double Y;

    public VuforiaNavTesting(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.updateLocation();
        X = robot.getLocationX();
        Y = robot.getLocationY();
        angle = robot.getRotation();
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Target Visible", robot.trackableVisible);

        engine.telemetry.addData("X", X);
        engine.telemetry.addData("Y", Y);
        engine.telemetry.addData("Angle", angle);
    }
}
