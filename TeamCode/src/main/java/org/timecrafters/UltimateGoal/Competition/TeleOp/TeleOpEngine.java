package org.timecrafters.UltimateGoal.Competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;

@TeleOp (name = "TeleOp",group = "comp")
public class TeleOpEngine extends CyberarmEngine {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        robot.wobbleGrabServo.setPosition(0);
        robot.webCamServo.setPosition(0);
        super.init();
    }

    @Override
    public void setup() {
        addState(new TeleOpState(robot));
    }

    @Override
    public void stop() {
        robot.deactivateVuforia();
        super.stop();
    }
}
