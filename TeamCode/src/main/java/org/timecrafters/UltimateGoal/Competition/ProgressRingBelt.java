package org.timecrafters.UltimateGoal.Competition;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

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
            targetPos = currentPos + Robot.RING_BELT_GAP;
            robot.ringBeltOn();
            robot.ringBeltStage += 1;
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (robot.ringBeltStage == 2) {
            targetPos = currentPos + 160;
            robot.ringBeltOn();
            robot.ringBeltStage += 1;
            prepLaunch = !robot.initLauncher;
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (robot.ringBeltStage > 2) {
            setHasFinished(true);
        }


    }

    @Override
    public void exec() {

        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        if (currentPos >= targetPos) {
            robot.ringBeltMotor.setPower(0);
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
            if(prepLaunch) {
                robot.launchMotor.setPower(Robot.LAUNCH_POWER);
            }

            setHasFinished(true);
        }

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
    }

}
