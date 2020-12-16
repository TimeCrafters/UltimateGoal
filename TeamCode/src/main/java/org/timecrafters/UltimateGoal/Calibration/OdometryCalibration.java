package org.timecrafters.UltimateGoal.Calibration;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;

import java.util.ArrayList;

public class OdometryCalibration extends CyberarmState {

    private Robot robot;
    private float rotation;
    private int currentTick;
    private double ticksPerDegree;

    public OdometryCalibration(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {

        if (engine.gamepad1.x) {
            double power = 0.1;
            robot.setDrivePower(power, -power, power, -power);
            currentTick = robot.encoderBack.getCurrentPosition();
            rotation = -robot.imu.getAngularOrientation().firstAngle;
            ticksPerDegree = currentTick/rotation;
        } else {
            robot.setDrivePower(0,0,0,0);
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("degrees", rotation);
        engine.telemetry.addData("ticks", currentTick);
        engine.telemetry.addData("ticks per degree", ticksPerDegree);
    }
}
