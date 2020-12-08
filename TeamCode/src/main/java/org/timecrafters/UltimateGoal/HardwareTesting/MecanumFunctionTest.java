package org.timecrafters.UltimateGoal.HardwareTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class MecanumFunctionTest extends CyberarmState {

    private Robot robot;
    private float leftJoystickDegrees;
    private double leftJoystickMagnitude;
    private float rightJoystickDegrees;
    private double rightJoystickMagnitude;
    private double powerFrontLeft=0;
    private double powerFrontRight=0;
    private double powerBackLeft=0;
    private double powerBackRight=0;

    private static double TURN_POWER_SCALE = 0.5;

    public MecanumFunctionTest(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.updateLocation();

        double leftJoystickX = engine.gamepad1.left_stick_x;
        double leftJoystickY = engine.gamepad1.left_stick_y;

        leftJoystickDegrees = (float) Math.toDegrees(Math.atan2(leftJoystickX, -leftJoystickY));
        leftJoystickMagnitude = Math.hypot(leftJoystickX, leftJoystickY);

        double rightJoystickX = engine.gamepad1.right_stick_x;
        double rightJoystickY = engine.gamepad1.right_stick_y;

        rightJoystickDegrees = (float) Math.toDegrees(Math.atan2(rightJoystickX, -rightJoystickY));
        rightJoystickMagnitude = Math.hypot(rightJoystickX, rightJoystickY);
//
        powerFrontLeft = 0;
        powerFrontRight = 0;
        powerBackLeft = 0;
        powerBackRight = 0;

        if (rightJoystickMagnitude == 0) {
            if (leftJoystickMagnitude !=0) {
                double[] powers = robot.getMecanumPowers(leftJoystickDegrees, leftJoystickMagnitude, leftJoystickDegrees);
                powerFrontLeft = powers[0];
                powerFrontRight = powers[1];
                powerBackLeft = powers[2];
                powerBackRight = powers[3];
            }
        } else {
            if (leftJoystickMagnitude == 0) {
                double[] powers =  robot.getFacePowers(rightJoystickDegrees, rightJoystickMagnitude);
                powerFrontLeft = TURN_POWER_SCALE * powers[0];
                powerFrontRight = TURN_POWER_SCALE * powers[1];
                powerBackLeft = TURN_POWER_SCALE * powers[0];
                powerBackRight = TURN_POWER_SCALE * powers[1];
            } else {
                double[] powers = robot.getMecanumPowers(leftJoystickDegrees, leftJoystickMagnitude, rightJoystickDegrees);
                powerFrontLeft = powers[0];
                powerFrontRight = powers[1];
                powerBackLeft = powers[2];
                powerBackRight = powers[3];
            }
        }

//        robot.record(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        robot.setDrivePower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

    }

    @Override
    public void telemetry() {

        engine.telemetry.addLine("Left Joystick");
        engine.telemetry.addData("Angle", leftJoystickDegrees);
        engine.telemetry.addData("Mag", leftJoystickMagnitude);

        engine.telemetry.addLine("Right Joystick");
        engine.telemetry.addData("Angle", rightJoystickDegrees);
        engine.telemetry.addData("Mag", rightJoystickMagnitude);

    }
}
