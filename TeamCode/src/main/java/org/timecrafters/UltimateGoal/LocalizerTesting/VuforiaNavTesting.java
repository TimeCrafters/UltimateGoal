package org.timecrafters.UltimateGoal.LocalizerTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class VuforiaNavTesting extends CyberarmState {

    private Robot robot;
    private float angle;
    private double X;
    private double Y;
    private float joystickDegrees;
    private double power;

    public VuforiaNavTesting(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        if (engine.gamepad1.x) {
            robot.syncWithVuforia();
        }
        
        robot.updateLocation();
//        robot.record();

        double joystickX = engine.gamepad1.right_stick_x;
        double joystickY = engine.gamepad1.right_stick_y;

        joystickDegrees = (float) Math.toDegrees(Math.atan2(joystickX, -joystickY));
        power = 0.3 * Math.hypot(joystickX, joystickY);

        robot.driveAtAngle(joystickDegrees, power);

        X = robot.getLocationX();
        Y = robot.getLocationY();
        angle = robot.getRotation();



    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Target Visible", robot.trackableVisible);
        engine.telemetry.addData("Odo X", robot.ticksToInches(X));
        engine.telemetry.addData("Odo Y", robot.ticksToInches(Y));

        engine.telemetry.addData("Joystick", joystickDegrees);
//        engine.telemetry.addData("Vis X", robot.ticksToInches(robot.visionX));
//        engine.telemetry.addData("Vis Y", robot.ticksToInches(robot.visionY));

        engine.telemetry.addData("Robot Angle", angle);
    }

}
