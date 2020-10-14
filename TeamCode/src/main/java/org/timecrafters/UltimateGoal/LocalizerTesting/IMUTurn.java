package org.timecrafters.UltimateGoal.LocalizerTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;

public class IMUTurn extends CyberarmState {

    private Robot robot;
    private String actionName;
    private String groupName;
    private double power;
    private float angleTarget;
    private int turnDirection;
    private float angleAllowance;
    private boolean useOptimalDirection;

    public IMUTurn(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.actionName = actionName;
        this.groupName = groupName;
    }

    @Override
    public void init() {
        power = robot.stateConfiguration.variable(groupName,actionName, "power").value();
        angleTarget = robot.stateConfiguration.variable(groupName,actionName, "angle").value();
        angleAllowance = robot.stateConfiguration.variable(groupName,actionName, "allowance").value();

        // turnDirection = robot.stateConfiguration.variable(groupName, actionName, "direction").value();

        turnDirection = 0;

        useOptimalDirection = (turnDirection == 0);
    }

    @Override
    public void start() {
        if (!robot.stateConfiguration.action(groupName,actionName).enabled) {
            setHasFinished(true);
        }
    }

    @Override
    public void exec() {
        robot.updateLocation();

        int OptimalDirection = getOptimalDirection(angleTarget, robot.getRotation());

        if (!useOptimalDirection && OptimalDirection == turnDirection){
            useOptimalDirection = true;
        }

        if (useOptimalDirection){
            turnDirection = OptimalDirection;
        }

        robot.setDrivePower(power * turnDirection,-power * turnDirection);

        if (robot.getRotation() > angleTarget - angleAllowance && robot.getRotation() < angleTarget + angleAllowance ) {
            robot.setDrivePower(0,0);
            setHasFinished(true);
        }



    }

    private int getOptimalDirection(float angleTarget, float angleCurrent){

        if (angleCurrent < 0) {
            angleCurrent += 360;
        }

        if (angleTarget < 0) {
            angleTarget += 360;
        }

        float degreeDifferance = angleTarget - angleCurrent;
        if (degreeDifferance > 180 || (degreeDifferance < 0 && degreeDifferance > -180)) {
            return -1;
        } else {
            return 1;
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("rotation", robot.getRotation());
    }
}
