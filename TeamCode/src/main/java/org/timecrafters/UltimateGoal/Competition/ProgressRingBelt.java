package org.timecrafters.UltimateGoal.Competition;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;

public class ProgressRingBelt extends CyberarmState {

    private Robot robot;
    private int targetPos;
    private boolean prepLaunch;
    private long stuckStartTime;

    public ProgressRingBelt(Robot robot) {
        this.robot = robot;
    }

    private void prep(){
        robot.ringBeltMotor.setTargetPosition(targetPos);
        robot.ringBeltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ringBeltMotor.setPower(Robot.RING_BELT_NORMAL_POWER);
        robot.ringBeltStage += 1;
        robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    @Override
    public void start() {
        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        if (robot.ringBeltStage < 2) {
            targetPos = currentPos + robot.ringBeltGap;
            prep();
        } else if (robot.ringBeltStage == 2) {
            targetPos = currentPos + 150;
            prep();
            prepLaunch = !robot.initLauncher;
        } else if (robot.ringBeltStage > 2) {
            setHasFinished(true);
        }


    }

    @Override
    public void exec() {

        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        if (currentPos >= targetPos) {
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
