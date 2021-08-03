package org.timecrafters.UltimateGoal.Competition.Demo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;

@TeleOp (name = "Demo: Tank",group = "demo")
public class DemoEngineTank extends CyberarmEngine {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_CLOSED);
        robot.webCamServo.setPosition(0);
        super.init();
    }

    @Override
    public void setup() {
        addState(new DemoStateTank(robot));
    }

    @Override
    public void stop() {
        robot.deactivateVuforia();
        robot.saveRecording();
        super.stop();
    }
}
