package org.timecrafters.UltimateGoal.Calibration;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class OdometryCalibration extends CyberarmState {

    private Robot robot;
    private float rotation = 0;
    private float rotationPrev = 0;
    private int currentTick;
    private double ticksPerDegreeClockwise;
    private double ticksPerDegreeCounterClockwise;
    private long timePrevious;
    private long timeChange;

    public OdometryCalibration(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.updateLocation();

        long timeCurrent = System.currentTimeMillis();
        timeChange = timeCurrent - timePrevious;

        if (timeChange >= 1200) {
            timePrevious = timeCurrent;


            double power = 0.25;
            float imu = -robot.imu.getAngularOrientation().firstAngle;
            rotation += robot.getRelativeAngle(imu, rotationPrev);
            rotationPrev = imu;

            currentTick = (robot.encoderRight.getCurrentPosition() - robot.encoderLeft.getCurrentPosition()) / 2;

            if (engine.gamepad1.x) {
                robot.setDrivePower(power, -power, power, -power);
                ticksPerDegreeClockwise = currentTick / rotation;
            } else if (engine.gamepad1.y) {
                robot.setDrivePower(-power, power, -power, power);
                ticksPerDegreeCounterClockwise = currentTick / rotation;
            } else {
                robot.setDrivePower(0, 0, 0, 0);
            }
        }

        if (engine.gamepad1.b) {
            robot.record("Clockwise : "+ticksPerDegreeClockwise);
            robot.record("Counter Clockwise : "+ticksPerDegreeCounterClockwise);
            robot.saveRecording();
            setHasFinished(true);
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Position ","("+round(robot.ticksToInches(robot.getLocationX()),0.1)+","+round(robot.ticksToInches(robot.getLocationY()),0.1)+")");
        engine.telemetry.addData("Rotation ", robot.getRotation());
//
//        engine.telemetry.addData("degrees", rotation);
        engine.telemetry.addData("ticks", robot.encoderRight.getCurrentPosition() );
        engine.telemetry.addData("Clockwise", ticksPerDegreeClockwise);
        engine.telemetry.addData("CounterClockwise", ticksPerDegreeCounterClockwise);
//        engine.telemetry.addData("forward", robot.forwardVector);
//        engine.telemetry.addData("sideways", robot.sidewaysVector);
    }

    private float round(double number,double unit) {
        return (float) (Math.floor(number/unit) * unit);
    }
}
