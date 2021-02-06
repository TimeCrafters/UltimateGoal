package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class ProgressRingBelt extends CyberarmState {

    private Robot robot;
    private int targetPos;
    private boolean useLaunchPrep;

    public ProgressRingBelt(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        useLaunchPrep = (robot.launchMotor.getPower() == 0);
    }

    @Override
    public void start() {
        int currentPos = robot.getBeltPos();
        if (robot.ringBeltStage < 2) {
            targetPos = robot.loopPos(currentPos + Robot.RING_BELT_GAP);
            robot.ringBeltOn();
            robot.ringBeltStage += 1;
        } else if (robot.ringBeltStage == 2) {
            targetPos = robot.loopPos(currentPos + 160);
            robot.ringBeltOn();
            robot.ringBeltStage += 1;
        }

    }

    @Override
    public void exec() {
        int currentPos = robot.getBeltPos();
        if (currentPos >= targetPos && currentPos < targetPos + Robot.RING_BELT_GAP) {
            robot.ringBeltMotor.setPower(0);

            if(useLaunchPrep) {
                robot.launchMotor.setPower(Robot.LAUNCH_POWER);
            }

            setHasFinished(true);
        }
    }

}
