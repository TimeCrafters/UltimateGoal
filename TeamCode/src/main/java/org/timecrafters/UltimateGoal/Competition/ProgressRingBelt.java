package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class ProgressRingBelt extends CyberarmState {

    private Robot robot;
    private int targetPos;
    private boolean prepLaunch;
    private long stuckStartTime;

    public ProgressRingBelt(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        if (robot.ringBeltStage < 2) {
            targetPos = robot.loopPos(currentPos + Robot.RING_BELT_GAP);
            robot.ringBeltOn();
            robot.ringBeltStage += 1;
        } else if (robot.ringBeltStage == 2) {
            targetPos = robot.loopPos(currentPos + 160);
            robot.ringBeltOn();
            robot.ringBeltStage += 1;
            prepLaunch = !robot.initLauncher;
        } else if (robot.ringBeltStage > 2) {
            setHasFinished(true);
        }


    }

    @Override
    public void exec() {
        if (robot.beltIsStuck() && childrenHaveFinished()) {
            long currentTime = System.currentTimeMillis();
            if (stuckStartTime == 0) {
                stuckStartTime = currentTime;
            } else if (currentTime - stuckStartTime >= robot.beltMaxStopTime) {
                addParallelState(new UnstickRingBelt(robot));
            }
        } else {
            stuckStartTime = 0;
        }

        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        if (currentPos >= targetPos) {
            robot.ringBeltMotor.setPower(0);

            if(prepLaunch) {
                robot.launchMotor.setPower(Robot.LAUNCH_POWER);
            }

            setHasFinished(true);
        }
    }

}
