package org.timecrafters.UltimateGoal.Competition.TeleOp;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Autonomous.DriveToCoordinates;
import org.timecrafters.UltimateGoal.Competition.Autonomous.Face;
import org.timecrafters.UltimateGoal.Competition.Pause;
import org.timecrafters.UltimateGoal.Competition.Robot;

import java.util.ArrayList;

public class powerShotsControl extends CyberarmState {

    private Robot robot;

    private double endGameX;
    private double endGameY;
    private float endGameRot;

    private double endGamePower;

    private int nextState = 0;

    private ArrayList<CyberarmState> states = new ArrayList<CyberarmState>();

    public powerShotsControl(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        endGameX = robot.stateConfiguration.variable("tele","_endGameStart","x").value();
        endGameY = robot.stateConfiguration.variable("tele","_endGameStart","y").value();
        endGameRot = robot.stateConfiguration.variable("tele","_endGameStart", "r").value();
        endGamePower = robot.stateConfiguration.variable("tele","_endGameStart", "power").value();

        endGameX = robot.inchesToTicks(endGameX);
        endGameY = robot.inchesToTicks(endGameY);
        endGameRot = (float) robot.inchesToTicks(endGameRot);

        states.add(new Pause(robot, "tele","_endGameStart"));
        states.add(new DriveToCoordinates(robot, "tele", "_pow1"));
        states.add(new Face(robot,"tele","_faceZero"));
        states.add(new LaunchControl(robot));
        states.add(new DriveToCoordinates(robot, "tele", "_pow2"));
        states.add(new Face(robot,"tele","_faceZero"));
        states.add(new LaunchControl(robot));
        states.add(new DriveToCoordinates(robot, "tele", "_pow3"));
        states.add(new Face(robot,"tele","_faceZero"));
        states.add(new LaunchControl(robot));
    }

    @Override
    public void start() {
        robot.setLocalization(endGameRot,endGameX,endGameY);
        robot.launchMotor.setPower(endGamePower);
    }

    @Override
    public void exec() {
        if (childrenHaveFinished()) {
            if (nextState < states.size()) {
                addParallelState(states.get(nextState));
                nextState += 1;
            } else {
                robot.launchMotor.setPower(0);
                setHasFinished(true);
            }
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Power shots children", childrenHaveFinished());
        for (CyberarmState state : children) {
            if (!state.getHasFinished()) {
                engine.telemetry.addLine("" + state.getClass());
            }
        }
    }
}
