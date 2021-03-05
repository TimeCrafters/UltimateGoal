package org.timecrafters.UltimateGoal.Competition;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

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
        targetPos = currentPos - robot.beltReverseTicks;
        robot.ringBeltMotor.setPower(-Robot.RING_BELT_POWER);
        robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
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
