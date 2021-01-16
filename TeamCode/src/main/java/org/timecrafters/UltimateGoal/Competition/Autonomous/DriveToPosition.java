package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class DriveToPosition extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    private double targetX;
    private double targetY;
    private double translation;
    private double tolerance;
    private double power;

    public DriveToPosition(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    @Override
    public void init() {
//        translation = robot.inchesToTicks(4);
        targetX = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName,actionName,"x").value());
        targetY = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName,actionName,"y").value());
        tolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName,actionName,"tolerance").value());
        power = robot.stateConfiguration.variable(groupName,actionName,"power").value();
    }

    @Override
    public void start() {

    }

    @Override
    public void exec() {
        robot.updateLocation();
//        robot.record();

        double distanceRemaining = Math.hypot(targetX - robot.getLocationX(), targetY - robot.getLocationY());

        //Math.pow(3, distanceRemaining - translation) + 0.3;

//            if (power > 0.65) {
//                power = 0.65;
//            }

        if (distanceRemaining < tolerance) {
            robot.setDrivePower(0, 0, 0, 0);
            setHasFinished(true);
        } else {
//            robot.driveAtAngle(robot.getAngleToPosition(targetX, targetY), power);
        }

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Target Visible", robot.trackableVisible);
        engine.telemetry.addData("Odo X", robot.ticksToInches(robot.getLocationX()));
        engine.telemetry.addData("Odo Y", robot.ticksToInches(robot.getLocationY()));
        engine.telemetry.addData(" Rotation", robot.getRotation());
    }
}
