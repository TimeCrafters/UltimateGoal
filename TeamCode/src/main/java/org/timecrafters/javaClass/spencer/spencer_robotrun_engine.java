package org.timecrafters.javaClass.spencer;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.javaClass.aubrey.AubreyFirstState;
import org.timecrafters.javaClass.samples.SampleRobot;
import org.timecrafters.javaClass.spencer.SpencerFirstState;

@TeleOp(name = "Spencer: runrobot", group = "spencer")
public class spencer_robotrun_engine extends CyberarmEngine {

    SampleRobot robot;
    @Override
    public void init() {
        robot = new SampleRobot(hardwareMap);
        robot.initHardware();
        robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        super.init();
    }

    @Override
    public void setup() {
        addState(new SpencerFirstState(robot));
    }
}
