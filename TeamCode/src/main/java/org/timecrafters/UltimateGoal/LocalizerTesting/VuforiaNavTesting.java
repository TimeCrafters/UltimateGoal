package org.timecrafters.UltimateGoal.LocalizerTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class VuforiaNavTesting extends CyberarmState {

    private Robot robot;
    private float angle;
    private double X;
    private double Y;
    private double targetX;
    private double targetY;

    public VuforiaNavTesting(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        targetX = robot.inchesToTicks(24);
        targetY = robot.inchesToTicks(24);
    }

    @Override
    public void exec() {
        robot.setDrivePower(-0.5 * engine.gamepad1.left_stick_y, -0.5 * engine.gamepad1.right_stick_y);
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

        engine.telemetry.addData("Robot Angle", angle);
        engine.telemetry.addData("Raw Angle", robot.rawAngle);
        engine.telemetry.addData("Angle to Target", robot.getAngleToPosition(targetX,targetY));
    }
}
