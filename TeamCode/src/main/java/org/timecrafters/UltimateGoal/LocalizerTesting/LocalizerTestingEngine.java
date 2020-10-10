package org.timecrafters.UltimateGoal.LocalizerTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Robot;

@Autonomous (name = "Localizer Test")
public class LocalizerTestingEngine extends CyberarmEngine {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        super.init();
    }

    @Override
    public void setup() {
        addState(new IMUDrive(robot,"group", "010_drive"));
        addState(new IMUTurn(robot,"group", "020_turn"));
        addState(new IMUDrive(robot,"group", "030_drive"));
        addState(new IMUTurn(robot,"group", "040_turn"));
    }
}
