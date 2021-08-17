package org.timecrafters.javaClass.spencer;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.javaClass.samples.SampleRobot;

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
