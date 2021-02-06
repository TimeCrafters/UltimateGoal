package org.timecrafters.UltimateGoal.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;

@Autonomous (name = "Autonomous")
public class AutoEngine extends CyberarmEngine {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        super.init();
    }

    @Override
    public void setup() {
        //drive to view
        addState(new DriveToCoordinates(robot, "auto", "001_0"));
        addState(new DriveToCoordinates(robot, "auto", "001_0"));

    }
}
