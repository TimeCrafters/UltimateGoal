package org.timecrafters.UltimateGoal.Competition.Demo;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.UltimateGoal.Competition.TeleOp.Player1;
import org.timecrafters.UltimateGoal.Competition.TeleOp.Player2;

public class DemoControl extends CyberarmState {

    private Robot robot;

    public DemoControl(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
//        addParallelState(new Demo1(robot));
//        addParallelState(new Demo2(robot));
    }

    @Override
    public void exec() {

    }

    @Override
    public void telemetry() {

        engine.telemetry.addLine("Location");
        engine.telemetry.addData("Position ","("+round(robot.ticksToInches(robot.getLocationX()),0.1)+","+round(robot.ticksToInches(robot.getLocationY()),0.1)+")");
        engine.telemetry.addData("Rotation ", robot.getRotation());
    }

    private float round(double number,double unit) {
        return (float) (Math.floor(number/unit) * unit);
    }

}
