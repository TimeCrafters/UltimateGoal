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
    }

    @Override
    public void start() {
        angleTarget=robot.getRotation();
    }

    @Override
    public void exec() {

        robot.updateLocation();

        if (Math.abs(robot.encoderRight.getCurrentPosition()) > tickTarget) {
            robot.encoderRight.setPower(0);
            robot.encoderLeft.setPower(0);
            setHasFinished(true);
        } else {

            angleRelative = robot.getRelativeAngle(angleTarget, robot.getRotation());

            double turnPowerCorrection = Math.pow(0.03 * angleRelative, 3) + 0.01 * angleRelative;

            double rightPower = power + turnPowerCorrection;
            double leftPower = power - turnPowerCorrection;

            double powerAdjust = ((2 * power) / (Math.abs(leftPower) + Math.abs(rightPower)));

            robot.encoderRight.setPower(rightPower * powerAdjust);
            robot.encoderLeft.setPower(leftPower * powerAdjust);
        }
    }
}
