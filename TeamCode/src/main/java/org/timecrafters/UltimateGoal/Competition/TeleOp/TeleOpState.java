package org.timecrafters.UltimateGoal.Competition.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Autonomous.DriveToCoordinates;
import org.timecrafters.UltimateGoal.Competition.Autonomous.FindWobbleGoal;
import org.timecrafters.UltimateGoal.Competition.Launch;
import org.timecrafters.UltimateGoal.Competition.ProgressRingBelt;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class TeleOpState extends CyberarmState {

    private Robot robot;

    public TeleOpState(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        addParallelState(new Player1(robot));
        addParallelState(new Player2(robot));
    }

    @Override
    public void exec() {
//        setHasFinished(childrenHaveFinished());
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
