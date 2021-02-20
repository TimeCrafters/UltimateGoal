package org.timecrafters.UltimateGoal.Competition.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Launch;
import org.timecrafters.UltimateGoal.Competition.ProgressRingBelt;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.UltimateGoal.Competition.WobbleArm;
import org.timecrafters.UltimateGoal.Competition.WobbleGrab;

public class TeleOpState extends CyberarmState {

    private Robot robot;
    private float leftJoystickDegrees;
    private double leftJoystickMagnitude;
    private float rightJoystickDegrees;
    private double rightJoystickMagnitude;
    private double rightJoystickMagnitudePrevious;

    private float faceDirection = 0;

    private double faceControlThreshold;
    private float cardinalSnapping;
    private float pairSnapping;



    private double[] powers = {0,0,0,0};
    private double drivePower = 1;
    private static final double TURN_POWER = 1;
    private static final double LAUNCH_TOLERANCE_FACE = 1;
    private Launch launchState;
    private boolean launching;
    private ProgressRingBelt ringBeltState;
    private boolean CollectorOn;
    private boolean rbPrev;
    private boolean yPrev;
    private boolean xPrev;
    private boolean bPrev;
    private boolean lbPrev;
    private boolean wobbleArmUp = false;
    private boolean wobbleGrabOpen = false;
    private boolean emergencyLaunchPrev;
    private double beltPowerPrev;
    private boolean reverseBeltPrev;
    private int manualArmHoldPos;
    private boolean manualArmHold;

    private boolean launchInput = false;


    public TeleOpState(Robot robot) {
        this.robot = robot;
    }

    private double LAUNCH_TOLERANCE_POS;

    @Override
    public void init() {
        cardinalSnapping = robot.stateConfiguration.variable("tele","control", "cardinalSnapping").value();
        pairSnapping = robot.stateConfiguration.variable("tele","control", "pairSnapping").value();
        faceControlThreshold = robot.stateConfiguration.variable("tele","control", "faceControlT").value();
        robot.wobbleArmMotor.setTargetPosition(0);
        robot.wobbleArmMotor.setPower(0.5);
        LAUNCH_TOLERANCE_POS = robot.inchesToTicks(3);
        robot.resetRotation(180);
    }

    @Override
    public void exec() {
        robot.updateLocation();
        robot.record(""+robot.ticksToInches(robot.getLocationX())+", "+robot.ticksToInches(robot.getLocationY()));

        boolean lb = engine.gamepad1.left_stick_button;
        if (lb && !lbPrev) {
            if (drivePower == 1) {
                drivePower = 0.5;
            } else {
                drivePower = 1;
            }
        }
        lbPrev = lb;

        double[] powers = {0,0,0,0};

        boolean y = engine.gamepad1.y;

        if (y) {

            //Drive to Launch Pos
            double distanceToTarget = Math.hypot(robot.launchPositionX - robot.getLocationX(), robot.launchPositionY - robot.getLocationY());
            if (distanceToTarget > LAUNCH_TOLERANCE_POS) {
                powers = robot.getMecanumPowers(robot.getAngleToPosition(robot.launchPositionX, robot.launchPositionY), drivePower, robot.launchRotation);

            } else if (Math.abs(robot.getRelativeAngle(robot.launchRotation,robot.getRotation())) > LAUNCH_TOLERANCE_FACE) {
                double[] facePowers =  robot.getFacePowers(robot.launchRotation, TURN_POWER);
                powers = new double[]{facePowers[0], facePowers[1], facePowers[0], facePowers[1]};
            } else {

            }

        } else {
            //Normal Driver Controls

            double leftJoystickX = engine.gamepad1.left_stick_x;
            double leftJoystickY = engine.gamepad1.left_stick_y;

            leftJoystickDegrees = robot.getRelativeAngle(90,  (float) Math.toDegrees(Math.atan2(leftJoystickX, -leftJoystickY)));
            leftJoystickMagnitude = Math.hypot(leftJoystickX, leftJoystickY);

            double rightJoystickX = engine.gamepad1.right_stick_x;
            double rightJoystickY = engine.gamepad1.right_stick_y;

            rightJoystickDegrees = robot.getRelativeAngle (90, (float) Math.toDegrees(Math.atan2(rightJoystickX, -rightJoystickY)));
            rightJoystickMagnitude = Math.hypot(rightJoystickX, rightJoystickY);

            //if the driver is letting go of the face joystick, the robot should maintain it's current face direction.
            if (rightJoystickMagnitude > faceControlThreshold || rightJoystickMagnitude - rightJoystickMagnitudePrevious > 0) {

                //if the joystick is close to one of the cardinal directions, it is set to exactly the cardinal direction
                faceDirection = snapToCardinal(rightJoystickDegrees,cardinalSnapping,0);
            }
            rightJoystickMagnitudePrevious = rightJoystickMagnitude;

            //allows the the driver to indicate which direction the robot is currently looking so
            //so that the controller and robot can be re-synced in the event of a bad initial
            //position.
            if (engine.gamepad1.right_stick_button) {
                robot.resetRotation(faceDirection);
            }

            if (leftJoystickMagnitude == 0) {
                double[] facePowers =  robot.getFacePowers(faceDirection,  TURN_POWER);
                powers = new double[]{facePowers[0], facePowers[1], facePowers[0], facePowers[1]};
            } else {
                //drives the robot in the direction of the move joystick while facing the direction
                //of the look joystick. if the move direction is almost aligned/perpendicular to the
                //look joystick,
                powers = robot.getMecanumPowers(snapToCardinal(leftJoystickDegrees,pairSnapping,faceDirection), drivePower, faceDirection);
            }

        }

        robot.setDrivePower(powers[0],powers[1],powers[2],powers[3]);

        this.powers = powers;

        if (childrenHaveFinished()) {
            robot.collectionMotor.setPower(engine.gamepad2.right_trigger);
        } else {
            robot.collectionMotor.setPower(0);
        }

        boolean rb = engine.gamepad2.right_bumper;
        if (rb && !rbPrev && childrenHaveFinished()) {
            addParallelState(new ProgressRingBelt(robot));
        }
        rbPrev = rb;

        boolean y2 = engine.gamepad2.y;
        if (y2 && !yPrev && childrenHaveFinished()) {
            launchState = new Launch(robot);
            addParallelState(launchState);
        }
        yPrev = y2;

        boolean x = engine.gamepad2.x;
        if (x && !xPrev) {
            wobbleGrabOpen = !wobbleGrabOpen;
            if (wobbleGrabOpen) {
                robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_MAX);
            } else {
                robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_MAX * 0.1 );
            }
        }
        xPrev = x;

        boolean b = engine.gamepad2.b;
        if (b && !bPrev) {
            wobbleArmUp = !wobbleArmUp;
            if (wobbleArmUp) {
                robot.wobbleArmMotor.setTargetPosition(550);
            } else {
                robot.wobbleArmMotor.setTargetPosition(0);
            }
        }
        bPrev = b;

        boolean emergencyLaunch = engine.gamepad2.a;
        if (emergencyLaunch && !emergencyLaunchPrev) {
            if (robot.launchMotor.getPower() == 0) {
                robot.launchMotor.setPower(Robot.LAUNCH_POWER);
            } else {
                robot.launchMotor.setPower(0);
            }
        }
        emergencyLaunchPrev = emergencyLaunch;

        if (engine.gamepad2.dpad_up) {
            robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.wobbleArmMotor.setPower(0.5);
            manualArmHold = true;
        } else if (engine.gamepad2.dpad_down) {
            robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.wobbleArmMotor.setPower(-0.5);
            manualArmHold = true;
        } else if (manualArmHold) {
            manualArmHold = false;
            robot.wobbleArmMotor.setTargetPosition(robot.wobbleArmMotor.getCurrentPosition());
            robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        boolean reverseBelt = engine.gamepad2.left_bumper;
        if (reverseBelt && !reverseBeltPrev) {
               beltPowerPrev = robot.ringBeltMotor.getPower();
               robot.ringBeltMotor.setPower(-Robot.RING_BELT_POWER);
        }

        if (!reverseBelt && reverseBeltPrev) {
            robot.ringBeltMotor.setPower(beltPowerPrev);
        }

        reverseBeltPrev = reverseBelt;

    }

    @Override
    public void telemetry() {

       engine.telemetry.addData("childrenHaveFinished", childrenHaveFinished());
       for (CyberarmState state : children) {
           if (!state.getHasFinished()) {
               engine.telemetry.addLine("" + state.getClass());
           }
       }

       engine.telemetry.addData("wobble Arm Up", wobbleArmUp);

        engine.telemetry.addLine("Location");
        engine.telemetry.addData("Position ","("+round(robot.ticksToInches(robot.getLocationX()),0.1)+","+round(robot.ticksToInches(robot.getLocationY()),0.1)+")");
        engine.telemetry.addData("Rotation ", robot.getRotation());
    }

    private float round(double number,double unit) {
        return (float) (Math.floor(number/unit) * unit);
    }

    private float snapToCardinal(float angle, float snapping, float offset) {
        int o = (int) offset + 180;
        o %= 90;
        for (int cardinal = o-180; (cardinal <= 180+o && cardinal != angle); cardinal += 90) {
            if (angle >= cardinal-snapping && angle <= cardinal+snapping) {
                angle = cardinal;
            }
        }
        return angle;
    }


}
