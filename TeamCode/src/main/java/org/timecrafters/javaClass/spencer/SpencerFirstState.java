package org.timecrafters.javaClass.spencer;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class SpencerFirstState extends CyberarmState {

    //here, you'll find some of your variables. you can add more as you need them.
    private SampleRobot robot;
    private double rightStick;
    private double leftStick;
    private boolean leftBumper;
    private boolean rightBumper;
    private boolean y;
    private boolean a;
    private float rightTrigger;
    private float leftTrigger;
    private boolean x;
    private boolean b;

    //This is the constructor. It lets other code bits run use the code you put here
    public SpencerFirstState(SampleRobot robot) {
        this.robot = robot;
    }

    //This is a method. methods are bits of code that can be run elsewhere.
    //This one is set up to repeat every few milliseconds
    @Override
    public void exec() {

        /* game pad read*/
        rightStick = engine.gamepad1.right_stick_y;
        leftStick = engine.gamepad1.left_stick_y;
        leftBumper = engine.gamepad1.left_bumper;
        rightBumper = engine.gamepad1.right_bumper;
        y = engine.gamepad1.y;
        a = engine.gamepad1.a;
        x = engine.gamepad1.x;
        rightTrigger = engine.gamepad1.right_trigger;
        leftTrigger = engine.gamepad1.right_trigger;
        b = engine.gamepad1.b;
        /*this section is no bumpers... tank drive*/

        if (!leftBumper && !rightBumper) {
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            robot.driveFrontLeft.setPower(-leftStick);
            robot.driveFrontRight.setPower(-rightStick);
            robot.driveBackLeft.setPower(-leftStick);
            robot.driveBackRight.setPower(-rightStick);
        }
        /*
        if (x) {
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            robot.driveFrontLeft.setPower(-.5);
            robot.driveFrontRight.setPower(-.5);
            robot.driveBackLeft.setPower(-.5);
            robot.driveBackRight.setPower(-.5);
        }

        if (b){
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        robot.driveFrontLeft.setPower(-1);
        robot.driveFrontRight.setPower(-1);
        robot.driveBackLeft.setPower(-1);
        robot.driveBackRight.setPower(-1);
    }
        */

        /*this is left bumper section... strafe to the left*/
        if (leftBumper) {
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            robot.driveFrontLeft.setPower(-1);
            robot.driveFrontRight.setPower(1);
            robot.driveBackLeft.setPower(1);
            robot.driveBackRight.setPower(-1);
        }

        /* right bumper section ... strafe to the right*/
        else if (rightBumper) {
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            robot.driveFrontLeft.setPower(1);
            robot.driveFrontRight.setPower(-1);
            robot.driveBackLeft.setPower(-1);
            robot.driveBackRight.setPower(1);
        }

        /* y section ... when y is pressed fly wheel starts, when a is pressed fly wheel stops*/
        if (y) {
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            robot.launchMotor.setPower(1);
        } else if (a) {
            robot.launchMotor.setPower(0);
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }

        /*right Trigger section ... when right trigger is held collection wheels suck rings*/

        if (rightTrigger>=0.5){
            robot.collectionMotor.setPower(1);
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        /*left trigger ... when left trigger is held tracks moves ring, when trigger is let go track stops*/
         else if (leftTrigger>=0.5) {
        robot.ringBeltMotor.setPower(1);
        robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        }

    }
}
