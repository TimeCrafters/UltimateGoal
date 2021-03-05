package org.timecrafters.UltimateGoal.HardwareTesting;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;

import org.cyberarm.engine.V2.CyberarmState;


public class LEDTest extends CyberarmState {

    RevBlinkinLedDriver leds;

    @Override
    public void init() {
        leds = engine.hardwareMap.get(RevBlinkinLedDriver.class, "leds");
    }

    @Override
    public void exec() {
        setHasFinished(false);
       if (engine.gamepad1.x) {
           leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
       } else {
           leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN );
       }
    }

}
