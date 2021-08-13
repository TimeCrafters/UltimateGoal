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
    private boolean aPrev;
    private double drivepower = 1;

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
        leftTrigger = engine.gamepad1.left_trigger;
        b = engine.gamepad1.b;
        /*this section is no bumpers... tank drive*/

        if (!leftBumper && !rightBumper) {
            robot.driveFrontLeft.setPower(-leftStick);
            robot.driveFrontRight.setPower(-rightStick);
            robot.driveBackLeft.setPower(-leftStick);
            robot.driveBackRight.setPower(-rightStick);
        }

        boolean a = engine.gamepad1.a;
        if (engine.gamepad1.a && !aPrev) {
            if (drivepower == 1) {
                drivepower = 0.5;
            } else {
                drivepower = 1;
            }
            engine.gamepad1.a = aPrev;



            /*this is left bumper section... strafe to the left*/
            if (leftBumper) {
                robot.driveFrontLeft.setPower(-1);
                robot.driveFrontRight.setPower(1);
                robot.driveBackLeft.setPower(1);
                robot.driveBackRight.setPower(-1);
            }

            /* right bumper section ... strafe to the right*/
            else if (rightBumper) {
                robot.driveFrontLeft.setPower(1);
                robot.driveFrontRight.setPower(-1);
                robot.driveBackLeft.setPower(-1);
                robot.driveBackRight.setPower(1);
            } else {
                robot.driveFrontLeft.setPower(-leftStick);
                robot.driveFrontRight.setPower(-rightStick);
                robot.driveBackLeft.setPower(-leftStick);
                robot.driveBackRight.setPower(-rightStick);
            }
            /* y section ... when y is pressed fly wheel starts, when y is pressed again fly wheel stops*/

            boolean changed = false; //outside of loop
            if (engine.gamepad1.y && !changed) {
                if (robot.launchMotor.getPower() == 0) robot.launchMotor.setPower(1);
                else robot.launchMotor.setPower(0);
                changed = true;
            } else if (!engine.gamepad1.y) changed = false;

            /*right Trigger section ... when right trigger is held collection wheels suck rings*/

            boolean rightTriggerB = (rightTrigger >= 0.5);
            if (rightTriggerB) {
                robot.collectionMotor.setPower(1);
            } else {
                robot.collectionMotor.setPower(0);

            }

            if (engine.gamepad1.b) {
                robot.ringBeltMotor.setPower(-0.5);
            }
            /*left trigger ... when left trigger is held tracks moves ring, when trigger is let go track stops*/

            if (leftTrigger >= 0.5) {
                robot.ringBeltMotor.setPower(1);
            } else {
                robot.ringBeltMotor.setPower(0);
            }

            if (robot.launchMotor.getPower() == 1) {
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (leftTrigger >= 0.5) {
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (rightTriggerB) {
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            } else if (drivepower == 0.5){
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            } else {
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        }
    }
}
