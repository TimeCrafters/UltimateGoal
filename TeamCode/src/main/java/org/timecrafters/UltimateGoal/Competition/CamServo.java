package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class CamServo extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    private boolean open;
    private long waitTime = -1;

    public CamServo(Robot robot, String groupName, String actionName, boolean open) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
        this.open = open;
    }

    public CamServo(Robot robot, boolean armUp, long waitTime) {
        this.robot = robot;
        this.open = armUp;
        this.waitTime = waitTime;
    }

    @Override
    public void init() {
        if (waitTime == -1) {
            waitTime = robot.stateConfiguration.variable(groupName, actionName, "wait").value();
        }
    }

    @Override
    public void start() {
        if (open) {
            robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_MAX);
        } else {
            robot.wobbleGrabServo.setPosition(0);
        }
    }

    @Override
    public void exec() {
        setHasFinished(runTime() > waitTime);
    }

}
