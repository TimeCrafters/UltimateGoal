package org.timecrafters.UltimateGoal.Competition.PreInit;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class FindLimitSwitch extends CyberarmState {

    private Robot robot;

    public FindLimitSwitch(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.ringBeltMotor.setPower(Robot.RING_BELT_SLOW_POWER);
    }

    @Override
    public void exec() {
        if (robot.magnetSensor.isPressed()) {
            robot.ringBeltMotor.setPower(0);
            setHasFinished(true);
        }
    }
}
