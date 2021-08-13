package org.timecrafters.UltimateGoal.Calibration;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class CalibrateRingBeltLoop extends CyberarmState {

    private Robot robot;
    private float startingTick;
    private int currentTick;
    private double ticksPerLoop;
    private boolean limit;
    private boolean hasSeenLimit;
    private boolean limitPrevious;

    public CalibrateRingBeltLoop(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {

        currentTick = robot.ringBeltMotor.getCurrentPosition();
        limit = robot.magnetSensor.isPressed();

        if (engine.gamepad1.x || (engine.gamepad1.a && !limit)) {
            robot.ringBeltMotor.setPower(0.5);
        } else {
           robot.ringBeltMotor.setPower(0);
        }

        if (engine.gamepad1.y) {
            robot.ringBeltMotor.setPower(-0.1);
        } else {
            robot.ringBeltMotor.setPower(0);
        }

        if (engine.gamepad1.b) {
            startingTick = currentTick;
        }

        if (limit && !limitPrevious){
            if (hasSeenLimit) {
                ticksPerLoop = currentTick - startingTick;
            }
            startingTick = currentTick;
            hasSeenLimit = true;
        }
        limitPrevious = limit;
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("ticks", currentTick - startingTick);
        engine.telemetry.addData("limit swich", limit );
        engine.telemetry.addData("Clockwise", ticksPerLoop);
    }
}
