package org.timecrafters.javaClass.spencer;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class SpencerFirstState extends CyberarmState {

    //here, you'll find some of your variables. you can add more as you need them.
    private SampleRobot robot;
    private double rightStick;
    private double leftStick;
    private boolean leftBumper;
    private boolean rightBumper;
    //This is the constructor. It lets other code bits run use the code you put here
    public SpencerFirstState(SampleRobot robot) {
        this.robot = robot;
    }

    //This is a method. methods are bits of code that can be run elsewhere.
    //This one is set up to repeat every few milliseconds
    @Override
    public void exec() {
    rightStick = engine.gamepad1.right_stick_y;
    leftStick = engine.gamepad1.left_stick_y;
    leftBumper = engine.gamepad1.left_bumper;
    rightBumper = engine.gamepad1.right_bumper;

    robot.driveFrontLeft.setPower(-leftStick);
    robot.driveFrontRight.setPower(-rightStick);
    robot.driveBackLeft.setPower(-leftStick);
    robot.driveBackRight.setPower(-rightStick);
            if (leftBumper) {
                robot.driveFrontLeft.setPower(-1);
                robot.driveFrontRight.setPower(-1);
                robot.driveBackLeft.setPower(-1);
                robot.driveBackRight.setPower(-1);
            } else if (rightBumper) {
                robot.driveFrontLeft.setPower(1);
                robot.driveFrontRight.setPower(1);
                robot.driveBackLeft.setPower(1);
                robot.driveBackRight.setPower(1);
            }


    }
}
