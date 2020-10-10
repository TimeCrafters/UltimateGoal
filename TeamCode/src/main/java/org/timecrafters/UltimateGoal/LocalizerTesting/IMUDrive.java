package org.timecrafters.UltimateGoal.LocalizerTesting;

import android.util.Log;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.TimeCraftersConfigurationTool.backend.Backend;
import org.timecrafters.UltimateGoal.Robot;

public class IMUDrive extends CyberarmState {

    private Robot robot;
    private String actionName;
    private String groupName;
    private double power;
    private int tickTarget;
    private float angleRelative;
    private float angleTarget;
    private int tickStart;
    private long finishDelay;

    public IMUDrive(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.actionName = actionName;
        this.groupName = groupName;
    }

    @Override
    public void init() {
        Log.i("Config", Backend.instance().gsonForConfig().toJson(robot.stateConfiguration.getConfig()));

        power = robot.stateConfiguration.variable(groupName, actionName, "power").value();
        double inchesTarget = robot.stateConfiguration.variable(groupName, actionName, "inches").value();
        tickTarget = (int) robot.inchesToTicks(inchesTarget);
        angleTarget = robot.stateConfiguration.variable(groupName, actionName, "angle").value();
        finishDelay = robot.stateConfiguration.variable(groupName,actionName,"delay").value();
    }

    @Override
    public void start() {
        if (!robot.stateConfiguration.action(groupName,actionName).enabled) {
            setHasFinished(true);
        }

        angleTarget=robot.getRotation();
        tickStart = robot.encoderRight.getCurrentPosition();
    }

    @Override
    public void exec() {

        robot.updateLocation();

        int ticksTraveled = Math.abs( robot.encoderRight.getCurrentPosition()-tickStart);
        if (ticksTraveled > tickTarget) {
            robot.setDrivePower(0,0);
            sleep(finishDelay);
            setHasFinished(true);
        } else {

            angleRelative = robot.getRelativeAngle(angleTarget, robot.getRotation());

            double turnPowerCorrection = Math.pow(0.03 * angleRelative, 3) + 0.01 * angleRelative;

            double rightPower = power + turnPowerCorrection;
            double leftPower = power - turnPowerCorrection;

            double powerAdjust = ((2 * power) / (Math.abs(leftPower) + Math.abs(rightPower)));

            robot.setDrivePower(powerAdjust*leftPower, powerAdjust*rightPower);
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("Measured Values");
        engine.telemetry.addData("Y", robot.ticksToInches(robot.getLocationY()));
        engine.telemetry.addData("X", robot.ticksToInches(robot.getLocationX()));
        engine.telemetry.addLine();
        engine.telemetry.addData("Rotation", robot.getRotation());
        engine.telemetry.addLine();
        engine.telemetry.addLine("Total Travel");
        engine.telemetry.addData("Left", robot.ticksToInches(robot.traveledLeft));
        engine.telemetry.addData("Right", robot.ticksToInches(robot.traveledRight));

    }
}
