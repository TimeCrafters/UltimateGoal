package org.timecrafters.UltimateGoal.Competition.TeleOp;

/*
The Player2 state has all the controls for player two. This includes a lot of automation and
emergency features, for if the driver wants to take control unexpectedly
*/

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.timecrafters.UltimateGoal.Competition.Launch;
import org.timecrafters.UltimateGoal.Competition.ProgressRingBelt;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class Player2 extends CyberarmState {
    private Robot robot;

    private boolean rbPrev;
    private boolean yPrev;
    private boolean xPrev;
    private boolean bPrev;
    private boolean wobbleArmUp = false;
    private boolean wobbleGrabOpen = false;
    private boolean aPrev;
    private double beltPowerPrev;
    private DcMotor.RunMode runModePrev;
    private boolean lbPrev;
    private boolean manualArmHold;
    private int loops = 0;



    private boolean launchInput = false;

    public Player2(Robot robot) {
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
//        loops+=1;
//        robot.record(""+runTime()+", "+loops+", "+robot.getRotation());

        //Collector control
        double rt = engine.gamepad2.right_trigger;
        double lt = engine.gamepad2.left_trigger;
        if (rt < lt) {
            robot.collectionMotor.setPower(-lt);
        } else if (childrenHaveFinished()) {
            robot.collectionMotor.setPower(rt);
        } else {
            robot.collectionMotor.setPower(0);
        }

        //The childrenHaveFinished() method tracks if there are parallel states that are still
        //running.
        if (childrenHaveFinished()) {
            //belt progression control
            boolean rb = engine.gamepad2.right_bumper;
            if (rb && !rbPrev) {
                addParallelState(new ProgressRingBelt(robot));
            }
            rbPrev = rb;

            //main launch sequence control
            boolean y2 = engine.gamepad2.y;
            if (y2 && !yPrev) {
                addParallelState(new Launch(robot));
            }
            yPrev = y2;


            //special launch sequence for single shots
            boolean x = engine.gamepad2.x;
            if (x && !bPrev) {
                addParallelState(new LaunchControl(robot));
            }
            bPrev = x;
        }

        //manually control the wobble arm for when it's initialized in an unexpected position.
        double leftStickY = engine.gamepad2.left_stick_y;

        if (engine.gamepad2.dpad_right) {
            if (!robot.wobbleTouchSensor.isPressed()) {
                setArmMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.wobbleArmMotor.setPower(-0.2);
                robot.wobbleArmMotor.setTargetPosition(robot.wobbleArmMotor.getCurrentPosition());
            } else {
                setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.wobbleArmMotor.setTargetPosition(0);
                setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        } else if (leftStickY != 0 ) {
            setArmMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.wobbleArmMotor.setPower(0.15 * leftStickY);
            robot.wobbleArmMotor.setTargetPosition(robot.wobbleArmMotor.getCurrentPosition());
        } else {
            setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleArmMotor.setPower(0.5);
            if (engine.gamepad2.dpad_up) {
                robot.wobbleArmMotor.setTargetPosition(robot.wobbleUpPos);
            } else if (engine.gamepad2.dpad_down) {
                robot.wobbleArmMotor.setTargetPosition(robot.wobbleDownPos);
            } else if (engine.gamepad2.dpad_left) {
                robot.wobbleArmMotor.setTargetPosition(robot.wobbleDropPos );
            }
        }

        //manually toggle the launch wheel for emergencies
        boolean a = engine.gamepad2.a;
        if (a && !aPrev) {
            if (robot.launchMotor.getPower() == 0) {
                robot.launchMotor.setPower(Robot.LAUNCH_POWER);
            } else {
                robot.launchMotor.setPower(0);
            }
        }
        aPrev = a;




        //allows the driver to revers the belt in the event of a jam
        boolean lb = engine.gamepad2.left_bumper;
        if (lb && !lbPrev) {
            runModePrev = robot.ringBeltMotor.getMode();
            beltPowerPrev = robot.ringBeltMotor.getPower();
            robot.ringBeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.ringBeltMotor.setPower(-Robot.RING_BELT_SLOW_POWER);
        }

        if (!lb && lbPrev) {
            robot.ringBeltMotor.setPower(beltPowerPrev);
            robot.ringBeltMotor.setMode(runModePrev);
        }
        lbPrev = lb;

        if (engine.gamepad2.back) {
            for (CyberarmState state : children) {
                engine.stopState(state);
            }
        }

    }


    //This just checks if the wobble arm runmode is already the desired mode before setting it.
    //I don't know if this is actually helpful
    private void setArmMode(DcMotor.RunMode runMode) {
        if (robot.wobbleArmMotor.getMode() != runMode) {
             robot.wobbleArmMotor.setMode(runMode);
        }
    }


    @Override
    public void telemetry() {
//        engine.telemetry.addLine("belt");
//        engine.telemetry.addData("power", robot.ringBeltMotor.getPower());
//        engine.telemetry.addData("pos", robot.ringBeltMotor.getCurrentPosition());
//        engine.telemetry.addData("target", robot.ringBeltMotor.getTargetPosition());
//
//        engine.telemetry.addData("ring belt stage", robot.ringBeltStage);
//        engine.telemetry.addData("Touch Sensor Pressed", robot.wobbleTouchSensor.isPressed());
//        engine.telemetry.addData("  Sensor value", robot.wobbleTouchSensor.getValue());
        engine.telemetry.addData("Player 2 children", childrenHaveFinished());
        for (CyberarmState state : children) {
            if (!state.getHasFinished()) {
                engine.telemetry.addLine("" + state.getClass());
            }
        }
    }

}
