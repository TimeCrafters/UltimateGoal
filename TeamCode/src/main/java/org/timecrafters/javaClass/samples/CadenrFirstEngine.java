package org.timecrafters.javaClass.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.javaClass.spencer.SpencerFirstState;

@Autonomous (name = "Caden: First Program", group = "caden")
public class CadenrFirstEngine extends CyberarmEngine {
    
    SampleRobot robot;

    @Override
    public void init() {
        robot = new SampleRobot(hardwareMap);
        robot.initHardware();
        robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_CLOSED);
        super.init();
    }
    
    @Override
    public void setup() {
        addState(new SpencerFirstState(robot));
    }
}
