package org.timecrafters.UltimateGoal.Competition.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.UltimateGoal.Competition.UnstickRingBelt;

public class LaunchSingle extends CyberarmState {

    private Robot robot;
    boolean hasCycled;
    boolean detectedPass;
    private int targetPos;
    private boolean reduceConditionPrev;
    private double reducePos;
    private long stuckStartTime;

    public LaunchSingle(Robot robot) {
        this.robot = robot;
    }


    @Override
    public void start() {
        robot.ringBeltStage +=1;
        robot.ringBeltMotor.setTargetPosition(robot.ringBeltMotor.getCurrentPosition() + robot.ringBeltGap);
        robot.ringBeltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ringBeltMotor.setPower(Robot.RING_BELT_NORMAL_POWER);
    }

    @Override
    public void exec() {
        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        if (currentPos >= targetPos) {
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
