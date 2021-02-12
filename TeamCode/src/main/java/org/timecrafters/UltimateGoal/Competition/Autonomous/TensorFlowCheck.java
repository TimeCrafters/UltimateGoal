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

    }

    @Override
    public void exec() {

         if (manualPath == -1) {
             recognitions = robot.tfObjectDetector.getUpdatedRecognitions();

             if (recognitions != null) {

                 if (recognitions.size() == 1) {
                     Recognition recognition = recognitions.get(0);
                     String label = recognition.getLabel();
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

         if (runTime() >= checkTime) {
            if (path == 0) {
                wobblePosX = robot.stateConfiguration.variable("auto", "_goalPos0","x").value();
                wobblePosY = robot.stateConfiguration.variable("auto", "_goalPos0","y").value();
            }
            if (path == 1) {
                 wobblePosX = robot.stateConfiguration.variable("auto", "_goalPos1","x").value();
                 wobblePosY = robot.stateConfiguration.variable("auto", "_goalPos1","y").value();
            }
             if (path == 2) {
                 wobblePosX = robot.stateConfiguration.variable("auto", "_goalPos2","x").value();
                 wobblePosY = robot.stateConfiguration.variable("auto", "_goalPos2","y").value();
             }
         }

    }
}
