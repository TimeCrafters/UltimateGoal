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



    @Override
    public void init() {
        checkTime = robot.stateConfiguration.variable(group,action,"time").value();
    }

    @Override
    public void start() {

    }

    @Override
    public void exec() {
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

         if (runTime() >= checkTime) {
            if (checkTime == 0) {
                wobblePosX = robot.stateConfiguration.variable("auto", "_goalPos1","x").value();
            }
         }

    }
}
