package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class WobbleArm extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    private int suggestedPos;
    private int targetPos = -1;
    private long waitTime = -1;

    public WobbleArm(Robot robot, String groupName, String actionName, int targetPos) {
        this.robot = robot;
        this.groupName  = groupName;
        this.actionName = actionName;
        this.suggestedPos = targetPos;
    }

    public WobbleArm(Robot robot, int targetPos, long waitTime) {
        this.robot = robot;
        this.targetPos = targetPos;
        this.waitTime = waitTime;
    }

    @Override
    public void init() {
        if (waitTime == -1) {
            waitTime = robot.stateConfiguration.variable(groupName, actionName, "wait").value();
            targetPos = robot.stateConfiguration.variable(groupName,actionName,"armPos").value();
            if (targetPos == -1) {
                targetPos = suggestedPos;
            }
        }
    }

    @Override
    public void start() {
        robot.wobbleArmMotor.setPower(0.5);
        robot.wobbleArmMotor.setTargetPosition(targetPos);
    }

    @Override
    public void exec() {
        setHasFinished(runTime() > waitTime);
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("runTime", runTime());
        engine.telemetry.addData("target", targetPos);
        engine.telemetry.addData("wait", waitTime);
    }
}
