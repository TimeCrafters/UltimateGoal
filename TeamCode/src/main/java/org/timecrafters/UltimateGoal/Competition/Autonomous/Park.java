package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

import java.util.ArrayList;

public class Park extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    double parkY;
    float parkFaceAngle;
    double parkTolerance;
    double parkPower;
    long parkBrakeTime;
    double  parkDecelRange;
    double parkDecelMin;

    public Park(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    @Override
    public void init() {
        parkY = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName,actionName,"yPos").value());
        parkFaceAngle = robot.stateConfiguration.variable(groupName,actionName,"face").value();
        parkTolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto",actionName,"tolPos").value());
        parkPower = robot.stateConfiguration.variable(groupName,actionName,"power").value();
        parkBrakeTime = robot.stateConfiguration.variable(groupName,actionName,"brakeMS").value();
        parkDecelMin = robot.stateConfiguration.variable(groupName,actionName,"decelM").value();
        parkDecelRange = robot.stateConfiguration.variable(groupName,actionName,"decelR").value();
    }

    @Override
    public void start() {
        if (Math.abs(robot.getLocationY()) > robot.inchesToTicks(8)) {
            addParallelState(new DriveToCoordinates(robot, robot.getLocationX(), parkY, parkFaceAngle, parkTolerance, parkPower, parkBrakeTime,parkDecelRange,parkDecelMin));
        }
    }

    @Override
    public void exec() {
        setHasFinished(childrenHaveFinished());
    }

}
