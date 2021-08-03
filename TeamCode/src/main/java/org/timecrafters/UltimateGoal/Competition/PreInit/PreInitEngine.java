package org.timecrafters.UltimateGoal.Competition.PreInit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;
@Disabled
@Autonomous (name = "Load Rings")
public class PreInitEngine extends CyberarmEngine {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        super.init();
    }

    @Override
    public void setup() {
        addState(new FindLimitSwitch(robot));
        addState(new LoadRings(robot));
    }
}
