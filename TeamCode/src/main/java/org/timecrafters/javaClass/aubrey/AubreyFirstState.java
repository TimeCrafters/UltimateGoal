package org.timecrafters.javaClass.aubrey;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class AubreyFirstState extends CyberarmState {

    //here, you'll find some of your variables. you can add more as you need them.
    private SampleRobot robot;
    private double UwU;
    private double OwO;


    //This is the constructor. It lets other code bits run use the code you put here
    public AubreyFirstState(SampleRobot robot) {
        this.robot = robot;
    }

    //This is a method. methods are bits of code that can be run elsewhere. 
    //This one is set up to repeat every few milliseconds
    @Override
    public void exec() {
        UwU= -engine.gamepad1.left_stick_y ;
        OwO= -engine.gamepad1.right_stick_y;
        if(robot.wobbleTouchSensor.isPressed()) {
            double power = .5;
            robot.driveFrontLeft.setPower(-power);
            robot.driveFrontRight.setPower(power);
            robot.driveBackLeft.setPower(-power);
            robot.driveBackRight.setPower(power);


        }else {
            robot.driveFrontLeft.setPower(UwU);
            robot.driveBackRight.setPower(OwO);
            robot.driveBackLeft.setPower(OwO);
            robot.driveBackLeft.setPower(UwU);
        }


    }

    @Override
    public void telemetry() {
        engine. telemetry.addData("uMu",UwU);
        engine. telemetry. addData("OmO", OwO);

    }
}
