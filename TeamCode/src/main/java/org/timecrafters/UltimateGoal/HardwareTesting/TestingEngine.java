package org.timecrafters.UltimateGoal.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Robot;

@TeleOp (name = "Mecanum test", group = "test")
public class TestingEngine extends CyberarmEngine {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        super.init();
    }



    @Override
    public void setup() {
        addState(new EncoderTest(robot));
    }

    @Override
    public void stop() {
        robot.deactivateVuforia();
    }
}
