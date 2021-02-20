package org.timecrafters.UltimateGoal.Competition.PreInit;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.ProgressRingBelt;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class LoadRings extends CyberarmState {

    private Robot robot;
    private ProgressRingBelt ringBeltState;
    private int ringCount = 0;

    public LoadRings(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.collectionMotor.setPower(1);
    }

    @Override
    public void exec() {
        ringBeltState = new ProgressRingBelt(robot);
        if (engine.gamepad1.x && childrenHaveFinished()) {
            addParallelState(ringBeltState);

        }

        if (robot.ringBeltStage > 2) {
            robot.launchMotor.setPower(0);
            setHasFinished(true);
        }
    }
}
