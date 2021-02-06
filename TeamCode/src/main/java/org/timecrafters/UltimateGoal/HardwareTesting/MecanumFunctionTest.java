package org.timecrafters.UltimateGoal.HardwareTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.timecrafters.UltimateGoal.Competition.Robot;


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

    private double aVx = 0;
    private double aVy = 0;
    private double aVz = 0;

    private int powerStep = 5;
    private double POWER_SCALE = 0.5 ;
    private boolean toggleSpeedInput = false;

    public MecanumFunctionTest(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        Velocity startVelocity = new Velocity();
        startVelocity.xVeloc = 0;
        startVelocity.yVeloc = 0;
        startVelocity.zVeloc = 0;

        Position startPosition = new Position();
        startPosition.x = 0;
        startPosition.y = 0;
        startPosition.z = 0;

        robot.imu.startAccelerationIntegration(startPosition,startVelocity, 10);
    }

    @Override
    public void exec() {
        robot.updateLocation();
//        robot.record();/////////////

        AngularVelocity angularVelocity = robot.imu.getAngularVelocity();

        aVx = angularVelocity.xRotationRate;
        aVy = angularVelocity.yRotationRate;
        aVz = angularVelocity.zRotationRate;

        double leftJoystickX = engine.gamepad1.left_stick_x;
        double leftJoystickY = engine.gamepad1.left_stick_y;

        leftJoystickDegrees = (float) Math.toDegrees(Math.atan2(leftJoystickX, -leftJoystickY));
        leftJoystickMagnitude = Math.hypot(leftJoystickX, leftJoystickY);

        double rightJoystickX = engine.gamepad1.right_stick_x;
        double rightJoystickY = engine.gamepad1.right_stick_y;

        rightJoystickDegrees = (float) Math.toDegrees(Math.atan2(rightJoystickX, -rightJoystickY));
        rightJoystickMagnitude = Math.hypot(rightJoystickX, rightJoystickY);

        powerFrontLeft = 0;
        powerFrontRight = 0;
        powerBackLeft = 0;
        powerBackRight = 0;

        boolean a = engine.gamepad1.a;
        boolean b = engine.gamepad1.b;

        if (a && !toggleSpeedInput && POWER_SCALE < 1) {
            powerStep += 1;
            POWER_SCALE = powerStep * 0.1;
        }

        if (b && !toggleSpeedInput && POWER_SCALE > 0.1) {
            powerStep -= 1;
            POWER_SCALE = powerStep * 0.1;
        }

        toggleSpeedInput = a || b;

        if (rightJoystickMagnitude == 0) {
            if (leftJoystickMagnitude !=0) {
                double[] powers = robot.getMecanumPowers(leftJoystickDegrees, POWER_SCALE * leftJoystickMagnitude, leftJoystickDegrees);
                powerFrontLeft = powers[0];
                powerFrontRight = powers[1];
                powerBackLeft = powers[2];
                powerBackRight = powers[3];

            }
        } else {
            if (leftJoystickMagnitude == 0) {
                double[] powers =  robot.getFacePowers(rightJoystickDegrees, POWER_SCALE * rightJoystickMagnitude);
                powerFrontLeft = powers[0];
                powerFrontRight = powers[1];
                powerBackLeft = powers[0];
                powerBackRight = powers[1];
            } else {
                double[] powers = robot.getMecanumPowers(leftJoystickDegrees, POWER_SCALE * leftJoystickMagnitude, rightJoystickDegrees);
                powerFrontLeft = powers[0];
                powerFrontRight = powers[1];
                powerBackLeft = powers[2];
                powerBackRight = powers[3];
            }
        }

        robot.setDrivePower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);


    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Scale", POWER_SCALE);

        engine.telemetry.addLine("Angular Velocity");
        engine.telemetry.addData("X", aVx);
        engine.telemetry.addData("Y", aVy);
        engine.telemetry.addData("Z", aVz);

        engine.telemetry.addLine("Powers");
        engine.telemetry.addData("FL", robot.driveFrontLeft.getPower());
        engine.telemetry.addData("FR", robot.driveFrontRight.getPower());
        engine.telemetry.addData("BL", robot.driveBackLeft.getPower());
        engine.telemetry.addData("BR", robot.driveBackRight.getPower());
    }
}
