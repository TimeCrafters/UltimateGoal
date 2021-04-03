package org.timecrafters.UltimateGoal.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Autonomous.FindWobbleGoal;
import org.timecrafters.UltimateGoal.Competition.Robot;

@TeleOp (name = "Hardware test", group = "test")
public class TestingEngine extends CyberarmEngine {

    private Robot robot;

    @Override
    public void setup() {
        addState(new ServoPosTest());
    }

}
