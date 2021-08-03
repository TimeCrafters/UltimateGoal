package org.timecrafters.UltimateGoal.Competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Autonomous.DriveToCoordinates;
import org.timecrafters.UltimateGoal.Competition.Autonomous.Face;
import org.timecrafters.UltimateGoal.Competition.Robot;
@Disabled
@TeleOp (name = "TeleOp",group = "comp")
public class TeleOpEngine extends CyberarmEngine {

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
        addState(new TeleOpState(robot));


    }

    @Override
    public void stop() {
        robot.deactivateVuforia();
        robot.saveRecording();
        super.stop();
    }
}
