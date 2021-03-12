package org.timecrafters.UltimateGoal.Competition.Demo;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Launch;
import org.timecrafters.UltimateGoal.Competition.ProgressRingBelt;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class Demo2 extends CyberarmState {
    private Robot robot;

    private boolean rbPrev;
    private boolean yPrev;
    private boolean xPrev;
    private boolean bPrev;
    private boolean wobbleArmUp = false;
    private boolean wobbleGrabOpen = false;
    private boolean aPrev;
    private double beltPowerPrev;
    private boolean lbPrev;
    private boolean manualArmHold;

    private boolean launchInput = false;

    public Demo2(Robot robot) {
        this.robot = robot;
    }


    @Override
    public void init() {
        robot.wobbleArmMotor.setTargetPosition(0);
        robot.wobbleArmMotor.setPower(0.5);
    }

    @Override
    public void start() {
        robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
    }

    @Override
    public void exec() {

        //Collector control
        if (childrenHaveFinished()) {
            robot.collectionMotor.setPower(engine.gamepad1.right_trigger);
        } else {
            robot.collectionMotor.setPower(0);
        }

        //belt progression control
        boolean rb = engine.gamepad1.right_bumper;
        if (rb && !rbPrev && childrenHaveFinished()) {
            addParallelState(new ProgressRingBelt(robot));
        }
        rbPrev = rb;

        //launch sequence control
        boolean y2 = engine.gamepad1.y;
        if (y2 && !yPrev && childrenHaveFinished()) {
            addParallelState(new Launch(robot));
        }
        yPrev = y2;

        //toggles wobble grabber open and closed
        boolean x = engine.gamepad1.x;
        if (x && !xPrev) {
            wobbleGrabOpen = !wobbleGrabOpen;
            if (wobbleGrabOpen) {
                robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_MAX);
            } else {
                robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_MAX * 0.05 );
            }
        }
        xPrev = x;

        //toggles the wobble arm up and down.
        boolean b = engine.gamepad1.b;
        if (b && !bPrev) {
            wobbleArmUp = !wobbleArmUp;
            if (wobbleArmUp) {
                robot.wobbleArmMotor.setTargetPosition(550);
            } else {
                robot.wobbleArmMotor.setTargetPosition(0);
            }
        }
        bPrev = b;

        //manually toggle the launch wheel for emergencies
        boolean a = engine.gamepad1.a;
        if (a && !aPrev) {
            if (robot.launchMotor.getPower() == 0) {
                robot.launchMotor.setPower(Robot.LAUNCH_POWER);
            } else {
                robot.launchMotor.setPower(0);
            }
        }
        aPrev = a;

        //manually control the wobble arm for when it's initialized in an unexpected position.
        if (engine.gamepad1.dpad_up) {
            robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.wobbleArmMotor.setPower(0.5);
            manualArmHold = true;
        } else if (engine.gamepad1.dpad_down) {
            robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.wobbleArmMotor.setPower(-0.1);
            manualArmHold = true;
        } else if (manualArmHold) {
            manualArmHold = false;
            robot.wobbleArmMotor.setTargetPosition(robot.wobbleArmMotor.getCurrentPosition());
            robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //allows the driver to revers the belt in the event of a jam
        boolean lb = engine.gamepad1.left_bumper;
        if (lb && !lbPrev) {
            beltPowerPrev = robot.ringBeltMotor.getPower();
            robot.ringBeltMotor.setPower(-Robot.RING_BELT_POWER);
        }

        if (!lb && lbPrev) {
            robot.ringBeltMotor.setPower(beltPowerPrev);
        }

        lbPrev = lb;

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Player 2 children", childrenHaveFinished());
        for (CyberarmState state : children) {
            if (!state.getHasFinished()) {
                engine.telemetry.addLine("" + state.getClass());
            }
        }
    }

}
