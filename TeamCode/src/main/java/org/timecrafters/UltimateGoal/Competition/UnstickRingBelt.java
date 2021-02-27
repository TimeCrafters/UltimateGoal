package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class UnstickRingBelt extends CyberarmState {

    private Robot robot;
    private int targetPos;
    private double lastPower;

    public UnstickRingBelt(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        lastPower = robot.ringBeltMotor.getPower();
        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        targetPos = robot.loopPos(currentPos - robot.beltReverseTicks);
        robot.ringBeltMotor.setPower(-Robot.RING_BELT_POWER);

    }

    @Override
    public void exec() {
        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        if (currentPos < targetPos) {
            robot.ringBeltMotor.setPower(lastPower);
            setHasFinished(true);
        }
    }

}
