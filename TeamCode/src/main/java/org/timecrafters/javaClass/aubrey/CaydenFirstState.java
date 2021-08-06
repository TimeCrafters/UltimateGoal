package org.timecrafters.javaClass.aubrey;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class CaydenFirstState extends CyberarmState {

    //here, you'll find some of your variables. you can add more as you need them.
    private SampleRobot robot;

    //This is the constructor. It lets other code bits run use the code you put here
    public CaydenFirstState(SampleRobot robot) {
        this.robot = robot;
    }

    //This is a method. methods are bits of code that can be run elsewhere. 
    //This one is set up to repeat every few milliseconds
    @Override
    public void exec() {
        robot.launchMotor.setPower(engine.gamepad1.left_stick_y);
        robot.ringBeltMotor.setPower(engine.gamepad1.right_trigger
        );
        if (robot.limitSwitch.isPressed()) {
            robot.launchMotor.setPower(0.5);
        }else{
            robot.launchMotor.setPower(0);
        }
    }
}
