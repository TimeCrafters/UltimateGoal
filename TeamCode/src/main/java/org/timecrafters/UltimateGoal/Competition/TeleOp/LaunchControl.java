package org.timecrafters.UltimateGoal.Competition.TeleOp;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Launch;
import org.timecrafters.UltimateGoal.Competition.ProgressRingBelt;
import org.timecrafters.UltimateGoal.Competition.ResetRingBelt;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class LaunchControl extends CyberarmState {

    private Robot robot;
    private boolean ranLaunch;

    public LaunchControl(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {

        if (robot.ringBeltStage > 2) {
            if (robot.ringBeltStage > 4) {
                addParallelState(new ResetRingBelt(robot));
            } else {
                addParallelState(new LaunchSingle(robot));
            }
            ranLaunch = true;
        }
    }

    @Override
    public void exec() {
        if (childrenHaveFinished()) {
            if (!ranLaunch) {
                if (robot.ringBeltStage <= 2) {
                    addParallelState(new ProgressRingBelt(robot));
                } else {
                    addParallelState(new LaunchSingle(robot));
                    ranLaunch = true;
                }
            } else {
                setHasFinished(true);
            }
        }
    }

    @Override
    public void telemetry() {

        engine.telemetry.addData("Launch Control children", childrenHaveFinished());
        for (CyberarmState state : children) {
            if (!state.getHasFinished()) {
                engine.telemetry.addLine("" + state.getClass());
            }
        }
    }
}
