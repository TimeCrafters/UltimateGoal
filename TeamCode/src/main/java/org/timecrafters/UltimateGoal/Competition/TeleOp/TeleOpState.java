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

    private double[] powers = {0,0,0,0};
    private double drivePower = 1;
    private static final double TURN_POWER = 1;
    private boolean toggleSpeedInput = false;
    private Launch launchState;
    private boolean launching;
    private ProgressRingBelt ringBeltState;
    private boolean CollectorOn;
    private boolean xPrev;
    private boolean yPrev;
    private boolean aPrev;
    private boolean bPrev;
    private boolean rbPrev;
    private boolean wobbleArmUp = true;
    private boolean wobbleGrabOpen = false;


    private boolean launchInput = false;


    public TeleOpState(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {

    }

    @Override
    public void exec() {
        robot.updateLocation();

        double leftJoystickX = engine.gamepad1.left_stick_x;
        double leftJoystickY = engine.gamepad1.left_stick_y;

        leftJoystickDegrees = (float) Math.toDegrees(Math.atan2(leftJoystickX, -leftJoystickY));
        leftJoystickMagnitude = Math.hypot(leftJoystickX, leftJoystickY);

        double rightJoystickX = engine.gamepad1.right_stick_x;
        double rightJoystickY = engine.gamepad1.right_stick_y;

        rightJoystickDegrees = (float) Math.toDegrees(Math.atan2(rightJoystickX, -rightJoystickY));
        rightJoystickMagnitude = Math.hypot(rightJoystickX, rightJoystickY);

        if (leftJoystickMagnitude > 0.66) {
            drivePower = 1 ;
        } else {
            drivePower = 0.6;
        }

        double[] powers = {0,0,0,0};

        boolean y = engine.gamepad1.y;
        if (y) {

            //Launch Sequence

            double distanceToTarget = Math.hypot(Robot.LAUNCH_POSITION_X - robot.getLocationX(), Robot.LAUNCH_POSITION_Y - robot.getLocationY());
            if (distanceToTarget > Robot.LAUNCH_TOLERANCE_POS) {
                powers = robot.getMecanumPowers(robot.getAngleToPosition(Robot.LAUNCH_POSITION_X, Robot.LAUNCH_POSITION_Y), drivePower, Robot.LAUNCH_ROTATION);

            } else if (robot.getRelativeAngle(Robot.LAUNCH_ROTATION, robot.getRotation()) > Robot.LAUNCH_TOLERANCE_FACE) {

                launchState = new Launch(robot);
                addParallelState(launchState);

            } else {

                double[] facePowers =  robot.getFacePowers(Robot.LAUNCH_ROTATION, TURN_POWER);
                powers = new double[]{facePowers[0], facePowers[1], facePowers[0], facePowers[1]};

            }

        }

        if (!y || ( launchState != null && launchState.getHasFinished())) {

            //Normal Driver Controls

            if (rightJoystickMagnitude == 0) {
                if (leftJoystickMagnitude !=0) {
                    powers = robot.getMecanumPowers(leftJoystickDegrees, drivePower, leftJoystickDegrees);
                }
            } else {
                if (leftJoystickMagnitude == 0) {
                    double[] facePowers =  robot.getFacePowers(rightJoystickDegrees, TURN_POWER * rightJoystickMagnitude);
                    powers = new double[]{facePowers[0], facePowers[1], facePowers[0], facePowers[1]};
                } else {
                    powers = robot.getMecanumPowers(leftJoystickDegrees, drivePower, rightJoystickDegrees);
                }
            }
        }

        robot.setDrivePower(powers[0],powers[1],powers[2],powers[3]);

        this.powers = powers;


        boolean x = engine.gamepad2.x;
        if (x && !xPrev && childrenHaveFinished()) {
            if (CollectorOn) {
                robot.collectionMotor.setPower(0);
                CollectorOn = false;
            } else {
                robot.collectionMotor.setPower(1);
                CollectorOn = true;
            }
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
            addParallelState(new ProgressRingBelt(robot));
        }
        aPrev = a;

        boolean b = engine.gamepad2.b;
        if (b && !bPrev) {
            wobbleArmUp = !wobbleArmUp;
            addParallelState(new WobbleArm(robot, wobbleArmUp, 100));
        }
        bPrev = b;

        boolean rb = engine.gamepad2.right_bumper;
        if (rb && !rbPrev) {
            wobbleGrabOpen = !wobbleGrabOpen ;
            addParallelState(new WobbleGrab( robot, wobbleGrabOpen, 100));
        }
        rbPrev = rb;
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("Powers");
        for (double power : powers) {
            engine.telemetry.addData(" ", power);
        }


        engine.telemetry.addLine("Location");
        engine.telemetry.addData("Position ","("+round(robot.ticksToInches(robot.getLocationX()))+","+round(robot.ticksToInches(robot.getLocationY()))+")");
        engine.telemetry.addData("Rotation ", robot.getRotation());
        engine.telemetry.addData("totalV", robot.totalV);
    }

    private float round(double number) {
        return ((float) Math.floor(number*100)) / 100;
    }


}
