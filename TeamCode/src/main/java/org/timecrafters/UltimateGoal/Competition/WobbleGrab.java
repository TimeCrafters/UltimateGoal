package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class WobbleGrab extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    private boolean open;
    private long waitTime = -1;
    private boolean enabled;

    public WobbleGrab(Robot robot, String groupName, String actionName, boolean open) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
        this.open = open;
    }

    public WobbleGrab(Robot robot, boolean open, long waitTime) {
        this.robot = robot;
        this.open = open;
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
        if (enabled) {
            if (open) {
                robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_OPEN);
            } else {
                robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_CLOSED);
            }
        } else {
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
