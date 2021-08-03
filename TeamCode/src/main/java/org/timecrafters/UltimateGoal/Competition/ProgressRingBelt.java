package org.timecrafters.UltimateGoal.Competition;

/*
The ProgressRingBelt state is used in teleOp to automatically move the ring belt.
*/

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

        //The first two progressions should move to preprare for another ring.
        if (robot.ringBeltStage < 2) {
            targetPos = currentPos + robot.ringBeltGap;
            prep();

            //The third progression needs to only move forward enought to block the ring from
            //falling out
        } else if (robot.ringBeltStage == 2) {
            targetPos = currentPos + 150;
            prep();
            prepLaunch = !robot.initLauncher;

            //If the ring belt is already full, It does  allow another progression
        } else if (robot.ringBeltStage > 2) {
            setHasFinished(true);
        }


    }

    @Override
    public void exec() {

        //finished state when the target position is reached
        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        if (currentPos >= targetPos) {
            if(prepLaunch) {
                robot.launchMotor.setPower(Robot.LAUNCH_POWER);
            }

            setHasFinished(true);
        }

        //belt jam mitigation
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
