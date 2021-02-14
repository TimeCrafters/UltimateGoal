package org.timecrafters.UltimateGoal.Competition.TeleOp;

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
    private Launch launchState;
    private boolean launching;
    private ProgressRingBelt ringBeltState;
    private boolean CollectorOn;
    private boolean xPrev;
    private boolean yPrev;
    private boolean aPrev;
    private boolean bPrev;
    private boolean lbPrev;
    private boolean wobbleArmUp = true;
    private boolean wobbleGrabOpen = false;


    private boolean launchInput = false;


    public TeleOpState(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        cardinalSnapping = robot.stateConfiguration.variable("tele","control", "cardinalSnapping").value();
        pairSnapping = robot.stateConfiguration.variable("tele","control", "pairSnapping").value();
        faceControlThreshold = robot.stateConfiguration.variable("tele","control", "faceControlT").value();
    }

    @Override
    public void exec() {
        robot.updateLocation();
        robot.record(""+robot.ticksToInches(robot.getLocationX())+", "+robot.ticksToInches(robot.getLocationY()));

        double leftJoystickX = engine.gamepad1.left_stick_x;
        double leftJoystickY = engine.gamepad1.left_stick_y;

        leftJoystickDegrees = (float) Math.toDegrees(Math.atan2(leftJoystickX, -leftJoystickY));
        leftJoystickMagnitude = Math.hypot(leftJoystickX, leftJoystickY);

        double rightJoystickX = engine.gamepad1.right_stick_x;
        double rightJoystickY = engine.gamepad1.right_stick_y;

        rightJoystickDegrees = (float) Math.toDegrees(Math.atan2(rightJoystickX, -rightJoystickY));
        rightJoystickMagnitude = Math.hypot(rightJoystickX, rightJoystickY);

        //if the driver is letting go of the face joystick, the robot should maintain it's current face direction.
        if (rightJoystickMagnitude > faceControlThreshold || rightJoystickMagnitude - rightJoystickMagnitudePrevious > 0) {
            //if the joystick is close to one of the cardinal directions, it is set to exactly the cardinal direction
            faceDirection = snapToCardinal(rightJoystickDegrees,cardinalSnapping,0);
        }
        rightJoystickMagnitudePrevious = rightJoystickMagnitude;

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

            //Launch Sequence

            double distanceToTarget = Math.hypot(robot.launchPositionX - robot.getLocationX(), robot.launchPositionY - robot.getLocationY());
            if (distanceToTarget > Robot.LAUNCH_TOLERANCE_POS) {
                powers = robot.getMecanumPowers(robot.getAngleToPosition(robot.launchPositionX, robot.launchPositionY), drivePower, robot.launchRotation);

            } else if (robot.getRelativeAngle(robot.launchRotation, robot.getRotation()) > Robot.LAUNCH_TOLERANCE_FACE) {

                launchState = new Launch(robot);
                addParallelState(launchState);

            } else {

                double[] facePowers =  robot.getFacePowers(robot.launchRotation, TURN_POWER);
                powers = new double[]{facePowers[0], facePowers[1], facePowers[0], facePowers[1]};

            }

        }

        if (!y || ( launchState != null && launchState.getHasFinished())) {

            //Normal Driver Controls

            if (leftJoystickMagnitude == 0) {
                double[] facePowers =  robot.getFacePowers(faceDirection,  TURN_POWER);
                powers = new double[]{facePowers[0], facePowers[1], facePowers[0], facePowers[1]};
            } else {

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

        boolean x = engine.gamepad2.x;
        if (x && !xPrev && childrenHaveFinished()) {
            addParallelState(new ProgressRingBelt(robot));
        }
        xPrev = x;

        boolean y2 = engine.gamepad2.y;
        if (y2 && !yPrev && childrenHaveFinished()) {
            launchState = new Launch(robot);
            addParallelState(launchState);
        }
        yPrev = y2;

        boolean a = engine.gamepad2.a;
        if (a && !aPrev && childrenHaveFinished()) {
            wobbleGrabOpen = !wobbleGrabOpen;
            addParallelState(new WobbleGrab( robot, wobbleGrabOpen, 100));
        }
        aPrev = a;

        boolean b = engine.gamepad2.b;
        if (b && !bPrev && childrenHaveFinished()) {
            wobbleArmUp = !wobbleArmUp;
            addParallelState(new WobbleArm(robot, wobbleArmUp, 100));
        }
        bPrev = b;


        if (engine.gamepad2.right_bumper && engine.gamepad2.left_bumper) {
            robot.launchMotor.setPower(Robot.LAUNCH_POWER);
        }

    }

    @Override
    public void telemetry() {

       engine.telemetry.addData("childrenHaveFinished", childrenHaveFinished());
       for (CyberarmState state : children) {
           if (!state.getHasFinished()) {
               engine.telemetry.addLine("" + state.getClass());
           }
       }

       engine.telemetry.addData("Vision Z", robot.visionZ);

       engine.telemetry.addData("belt stage", robot.ringBeltStage);

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
