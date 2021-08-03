package org.timecrafters.UltimateGoal.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;
@Disabled
@TeleOp (name = "Calibration", group = "test")
public class CalibrationEngine extends CyberarmEngine {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        super.init();
    }


    @Override
    public void setup() {
        addState(new OdometryCalibration(robot));
    }
}
