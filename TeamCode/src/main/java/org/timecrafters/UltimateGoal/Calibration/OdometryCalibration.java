package org.timecrafters.UltimateGoal.Calibration;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;

public class OdometryCalibration extends CyberarmState {

    private Robot robot;
    private float rotation;
    private int currentTick;
    private double ticksPerDegreeClockwise;
    private double ticksPerDegreeCounterClockwise;

    public OdometryCalibration(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {

        double power = 0.1;
        rotation = -robot.imu.getAngularOrientation().firstAngle;
        currentTick = robot.encoderBack.getCurrentPosition();

        if (engine.gamepad1.x) {
            robot.setDrivePower(power, -power, power, -power);
            ticksPerDegreeClockwise = currentTick/rotation;
        } else if(engine.gamepad1.y) {
            robot.setDrivePower(-power, power, -power, power);
            ticksPerDegreeCounterClockwise = currentTick/rotation;
        } else {
            robot.setDrivePower(0,0,0,0);
        }

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("degrees", rotation);
        engine.telemetry.addData("ticks", currentTick);
        engine.telemetry.addData("Clockwise", ticksPerDegreeClockwise);
        engine.telemetry.addData("CounterClockwise", ticksPerDegreeCounterClockwise);
    }
}
