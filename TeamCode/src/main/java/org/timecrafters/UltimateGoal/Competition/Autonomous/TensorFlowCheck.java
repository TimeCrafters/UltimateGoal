package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.timecrafters.UltimateGoal.Competition.Robot;

import java.util.ArrayList;
import java.util.List;

public class TensorFlowCheck extends CyberarmState {

    private Robot robot;
    private String group;
    private String action;
    private List<Recognition> recognitions;
    private int path = 0;
    private long checkTime;
    public double wobblePosX;
    public double wobblePosY;
    private int manualPath;
    private String status;

    public TensorFlowCheck(Robot robot, String group, String action) {
        this.robot = robot;
        this.group = group;
        this.action = action;
    }

    @Override
    public void init() {
        checkTime = robot.stateConfiguration.variable(group,action,"time").value();
        manualPath = robot.stateConfiguration.variable(group,action,"path").value();
    }

    @Override
    public void start() {
        if (!robot.stateConfiguration.action(group,action).enabled) {
            manualPath = -1;
        } else {
            robot.tfObjectDetector.activate();
        }
    }

    @Override
    public void exec() {

        if (runTime() < checkTime) {
            if (manualPath != -1) {
                recognitions = robot.tfObjectDetector.getUpdatedRecognitions();

                if (recognitions != null) {
                    if (recognitions.size() == 1) {
                        String label = recognitions.get(0).getLabel();
                        if (label.equals("Single")) {
                            path = 1;
                        } else if (label.equals("Quad")) {
                            path = 2;
                        }
                    } else if (recognitions.size() > 1) {
                        path = 1;
                    }
                }

            } else {
                path = manualPath;
            }
        } else {
            robot.tfObjectDetector.deactivate();

            if (path == 0) {
                robot.wobbleScoreX = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto", "_goalPos0","x").value());
                robot.wobbleScoreY = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto", "_goalPos0","y").value());
            }
            if (path == 1) {
                robot.wobbleScoreX = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto", "_goalPos1","x").value());
                robot.wobbleScoreY = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto", "_goalPos1","y").value());
            }
             if (path == 2) {
                 robot.wobbleScoreX = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto", "_goalPos2","x").value());
                 robot.wobbleScoreY = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto", "_goalPos2","y").value());
             }
             // make the servo look up once we're done using tensorFlow
             robot.webCamServo.setPosition(0);
             robot.launchMotor.setPower(Robot.LAUNCH_POWER);

             setHasFinished(true);
         }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Chosen Path", path);
    }
}
