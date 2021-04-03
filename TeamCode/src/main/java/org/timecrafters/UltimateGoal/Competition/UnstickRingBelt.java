package org.timecrafters.UltimateGoal.Competition;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;

public class UnstickRingBelt extends CyberarmState {

    private Robot robot;
    private int targetPos;
    private double lastPower;
    private DcMotor.RunMode lastRunMode;

    public UnstickRingBelt(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        lastPower = robot.ringBeltMotor.getPower();
        lastRunMode = robot.ringBeltMotor.getMode();
        int currentPos = robot.ringBeltMotor.getCurrentPosition();
        targetPos = currentPos - robot.beltReverseTicks;
        robot.ringBeltMotor.setPower(-Robot.RING_BELT_SLOW_POWER);
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
