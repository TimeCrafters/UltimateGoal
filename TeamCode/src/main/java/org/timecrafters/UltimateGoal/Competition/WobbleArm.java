package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class WobbleArm extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    private boolean armUp;
    private long waitTime = -1;

    public WobbleArm(Robot robot, String groupName, String actionName, boolean armUp) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
        this.armUp = armUp;
    }

    public WobbleArm(Robot robot, boolean armUp, long waitTime) {
        this.robot = robot;
        this.armUp = armUp;
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
        robot.wobbleArmMotor.setPower(0.3);
        if (armUp) {
            robot.wobbleArmMotor.setTargetPosition(0);
        } else {
            robot.wobbleArmMotor.setTargetPosition(Robot.WOBBLE_ARM_DOWN);
        }
    }

    @Override
    public void exec() {
        setHasFinished(runTime() > waitTime);
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("runTime", runTime());
    }
}
