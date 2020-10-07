package org.timecrafters.UltimateGoal.LocalizerTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;

public class IMUTurn extends CyberarmState {

    private Robot robot;
    private String actionName;
    private String groupName;
    private double power;
    private double angleTarget;
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
        turnDirection = robot.stateConfiguration.variable(groupName, actionName, "direction").value();
        useOptimalDirection = (turnDirection==0);
    }

    @Override
    public void exec() {
        robot.updateLocation();

        int OptimalDirection;

        if (angleTarget > 180 + robot.getRotation()){
            OptimalDirection = -1;
        } else if (angleTarget < -180 + robot.getRotation()){
            OptimalDirection = 1;
        } else if (angleTarget > robot.getRotation()){
            OptimalDirection = 1;
        }else {
            OptimalDirection = -1;
        }

        if (!useOptimalDirection && OptimalDirection == turnDirection){
            useOptimalDirection = true;
        }

        if (useOptimalDirection){
            turnDirection = OptimalDirection;
        }

        robot.encoderLeft.setPower(-power * turnDirection);
        robot.encoderRight.setPower(power * turnDirection);

        if (robot.getRotation() > angleTarget - angleAllowance && robot.getRotation() < angleTarget + angleAllowance ) {
            robot.encoderRight.setPower(0);
            robot.encoderLeft.setPower(0);
            setHasFinished(true);
        }

    }
}
