package org.timecrafters.javaClass.samples;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.javaClass.aubrey.AubreyFirstState;
import org.timecrafters.javaClass.spencer.SpencerFirstState;

@TeleOp(name = "Spencer: Buttons", group = "spencer")
public class SpencerFirstEngine extends CyberarmEngine {
    
    SampleRobot robot;
    boolean yBeingPressed;
    @Override
    public void init() {
        robot = new SampleRobot(hardwareMap);
        robot.initHardware();
        robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        super.init();
        yBeingPressed = false;
    }
    
    @Override
    public void setup() {
        addState(new Spencer_buttons(robot,yBeingPressed));
    }
}
