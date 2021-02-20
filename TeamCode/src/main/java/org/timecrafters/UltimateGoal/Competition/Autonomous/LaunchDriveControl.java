package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class LaunchDriveControl extends CyberarmState {

    private Robot robot;
    private CyberarmState launchState;
    private CyberarmState driveState;
    private boolean checkConditionPrev;

    public LaunchDriveControl(Robot robot, CyberarmState launchState, CyberarmState driveState) {
        this.robot = robot;
        this.launchState = launchState;
        this.driveState = driveState;
    }

    @Override
    public void start() {
        addParallelState(launchState);
    }

    @Override
    public void exec() {
        boolean checkCondition = (robot.ringBeltStage > 3);
        if (checkCondition && !checkConditionPrev) {
            addParallelState(driveState);
        }
        checkConditionPrev = checkCondition;

        setHasFinished(childrenHaveFinished());
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("ring belt stage", robot.ringBeltStage);
        engine.telemetry.addData("check prev", checkConditionPrev);
        engine.telemetry.addData("drive  prev", checkConditionPrev);
    }
}
