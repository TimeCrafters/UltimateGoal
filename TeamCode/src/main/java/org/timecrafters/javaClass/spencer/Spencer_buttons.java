package org.timecrafters.javaClass.spencer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class Spencer_buttons extends CyberarmState {
    public Spencer_buttons(SampleRobot robot, boolean yBeingPressed) {
        this.robot = robot;
       this.yBeingPressed = yBeingPressed;
    }
    SampleRobot robot;
    boolean yBeingPressed;
    int Lights = 0;
    @Override
    public void exec() {

    if (engine.gamepad1.y && !yBeingPressed){
        yBeingPressed = true;
        Lights ++;
        if (Lights>4)Lights = 1;
        switch (Lights) {
            case 1:
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case 2:
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case 3:
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            case 4:
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                break;
        }


    }
    else if (!engine.gamepad1.y && yBeingPressed){
        yBeingPressed = false;
    }
    }

}
