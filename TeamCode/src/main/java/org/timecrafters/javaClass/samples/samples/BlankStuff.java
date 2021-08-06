package org.timecrafters.javaClass.samples.samples;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class BlankStuff extends CyberarmState {

    //here, you'll find some of your variables. you can add more as you need them.
    private SampleRobot robot;

    //This is the constructor. It lets other code bits run use the code you put here
    public BlankStuff(SampleRobot robot) {
        this.robot = robot;
    }

    //This is a method. methods are bits of code that can be run elsewhere. 
    //This one is set up to repeat every few milliseconds
    @Override
    public void exec() {

        double targetDistance = robot.inchesToTicks(12);
        if (robot.encoderLeft.getCurrentPosition() <= targetDistance) {
            robot.driveBackRight.setPower(1);
        }

    }
}
