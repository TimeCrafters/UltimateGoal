package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class Pause extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    private long waitTime = -1;
    private boolean enabled;

    public Pause(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    public Pause(Robot robot,  long waitTime) {
        this.robot = robot;
        this.waitTime = waitTime;
    }

    @Override
    public void init() {
        if (waitTime == -1) {
            enabled = robot.stateConfiguration.action(groupName, actionName).enabled;
            waitTime = robot.stateConfiguration.variable(groupName, actionName, "wait").value();
        }
    }

    @Override
    public void start() {
        if (!enabled) {
            setHasFinished(true);
        }
    }

    @Override
    public void exec() {
        setHasFinished(runTime() > waitTime);
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("runTime", runTime());
        engine.telemetry.addData("wait", waitTime);
    }

}
