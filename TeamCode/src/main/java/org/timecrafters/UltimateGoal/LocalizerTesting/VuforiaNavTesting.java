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
        robot.setDrivePower(-0.2 * engine.gamepad1.left_stick_y, -0.2 * engine.gamepad1.right_stick_y);
        robot.updateLocation();
        X = robot.getLocationX();
        Y = robot.getLocationY();
        angle = robot.getRotation();
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Target Visible", robot.trackableVisible);

        engine.telemetry.addData("Odo X", robot.ticksToInches(X));
        engine.telemetry.addData("Odo Y", robot.ticksToInches(Y));

        engine.telemetry.addData("Vis X", robot.visionX);
        engine.telemetry.addData("Vis Y", robot.visionY);

        engine.telemetry.addData("Angle", angle);
    }
}
