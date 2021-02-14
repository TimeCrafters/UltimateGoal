package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

import java.util.ArrayList;

public class FaceAndDrive extends CyberarmState {

    private Robot robot;
    private String groupName = "manual";
    private String actionName;
    private double xTarget;
    private double yTarget;
    private float faceAngle;
    private double tolerancePos;
    private double power;
    private long brakeTime;
    private float toleranceFace;

    public FaceAndDrive(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    @Override
    public void init() {

            xTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "xPos").value());
            yTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "yPos").value());
            power = robot.stateConfiguration.variable(groupName, actionName, "power").value();
            tolerancePos = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "tolPos").value());
            toleranceFace = robot.stateConfiguration.variable(groupName, actionName, "tolFace").value();
            brakeTime = robot.stateConfiguration.variable(groupName, actionName, "brakeMS").value();

            try {
                faceAngle = robot.stateConfiguration.variable(groupName, actionName, "face").value();
            } catch (NullPointerException e) {
                faceAngle = robot.getAngleToPosition(xTarget,yTarget);
            }


    }

    @Override
    public void start() {
        addState(new Face(robot, faceAngle, toleranceFace, power, brakeTime));
        addState(new DriveToCoordinates(robot,xTarget,yTarget,faceAngle,tolerancePos,power,brakeTime));
    }

    @Override
    public void exec() {
        setHasFinished(true);
    }
}
