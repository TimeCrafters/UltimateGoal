package org.timecrafters.javaClass.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.javaClass.aubrey.AubreyFirstState;
import org.timecrafters.javaClass.aubrey.dance;

@Autonomous(name = "Aubrey: First Program", group = "aubrey")
public class AubreyFirstEngine extends CyberarmEngine {

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
        addState(new dance(robot));
    }
}
