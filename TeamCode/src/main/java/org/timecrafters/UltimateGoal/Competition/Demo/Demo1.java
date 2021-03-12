package org.timecrafters.UltimateGoal.Competition.Demo;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Autonomous.DriveToCoordinates;
import org.timecrafters.UltimateGoal.Competition.Autonomous.FindWobbleGoal;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class Demo1 extends CyberarmState {
    private Robot robot;

    //normal drive control
    private float leftJoystickDegrees;
    private double leftJoystickMagnitude;
    private float rightJoystickDegrees;
    private double rightJoystickMagnitude;
    private double rightJoystickMagnitudePrevious;

    private double faceControlThreshold;
    private float cardinalSnapping;
    private float pairSnapping;

    private float faceDirection = 0;
    private double[] powers = {0,0,0,0};
    private double drivePower = 1;
    private boolean lbPrev;

    //find wobble goal control
    private FindWobbleGoal findWobbleGoal;
    private boolean runNextFindWobble;
    private boolean findWobbleInputPrev;

    //Drive to launch control
    private DriveToCoordinates driveToLaunch;
    private boolean runNextDriveToLaunch;
    private boolean driveToLaunchInputPrev;

    private double launchTolerance;
    private double launchPower;
    private long launchBrakeTime;

    private float launchAngleGoal;
    private float launchAnglePower1;
    private float launchAnglePower2;
    private float launchAnglePower3;

    public Demo1(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        cardinalSnapping = robot.stateConfiguration.variable("tele","control", "cardinalSnapping").value();
        pairSnapping = robot.stateConfiguration.variable("tele","control", "pairSnapping").value();
        faceControlThreshold = robot.stateConfiguration.variable("tele","control", "faceControlT").value();

    }

    @Override
    public void start() {
        faceDirection = robot.getRotation();
    }

    @Override
    public void exec() {
        robot.updateLocation();

        boolean lb = engine.gamepad1.left_stick_button;
        if (lb && !lbPrev) {
            if (drivePower == 1) {
                drivePower = 0.5;
            } else {
                drivePower = 1;
            }
        }
        lbPrev = lb;

        runNextFindWobble = (findWobbleGoal == null || findWobbleGoal.getHasFinished());

        boolean findWobbleInput = engine.gamepad1.dpad_up;
        if (findWobbleInput) {
            if (runNextFindWobble && !findWobbleInputPrev) {
                findWobbleGoal = new FindWobbleGoal(robot, "auto", "08_0");
                addParallelState(findWobbleGoal);
            }
            faceDirection = robot.getRotation();
        } else if (!runNextFindWobble) {
            findWobbleGoal.setHasFinished(true);
        }
        findWobbleInputPrev = findWobbleInput;

        if (childrenHaveFinished()) {
            //Normal Driver Controls

            double leftJoystickX = engine.gamepad1.left_stick_x;
            double leftJoystickY = engine.gamepad1.left_stick_y;

            leftJoystickDegrees = robot.getRelativeAngle(90, (float) Math.toDegrees(Math.atan2(leftJoystickX, -leftJoystickY)));
            leftJoystickMagnitude = Math.hypot(leftJoystickX, leftJoystickY);

            double rightJoystickX = engine.gamepad1.right_stick_x;
            double rightJoystickY = engine.gamepad1.right_stick_y;

            rightJoystickDegrees = robot.getRelativeAngle(90, (float) Math.toDegrees(Math.atan2(rightJoystickX, -rightJoystickY)));
            rightJoystickMagnitude = Math.hypot(rightJoystickX, rightJoystickY);

            //allows the the driver to indicate which direction the robot is currently looking so
            //so that the controller and robot can be re-synced in the event of a bad initial
            //position.
            if (engine.gamepad1.right_stick_button) {
                robot.setLocalization(rightJoystickDegrees, robot.getLocationX(), robot.getLocationY());
                faceDirection = rightJoystickDegrees;
            }

            //if the driver is letting go of the face joystick, the robot should maintain it's current face direction.
            if (rightJoystickMagnitude > faceControlThreshold || rightJoystickMagnitude - rightJoystickMagnitudePrevious > 0) {

                //if the joystick is close to one of the cardinal directions, it is set to exactly the cardinal direction
                faceDirection = snapToCardinal(rightJoystickDegrees, cardinalSnapping, 0);
            }
            rightJoystickMagnitudePrevious = rightJoystickMagnitude;


            if (leftJoystickMagnitude == 0) {
                double[] facePowers = robot.getFacePowers(faceDirection, 0.4);
                powers = new double[]{facePowers[0], facePowers[1], facePowers[0], facePowers[1]};
            } else {
                //drives the robot in the direction of the move joystick while facing the direction
                //of the look joystick. if the move direction is almost aligned/perpendicular to the
                //look joystick,
                powers = robot.getMecanumPowers(snapToCardinal(leftJoystickDegrees, pairSnapping, faceDirection), drivePower, faceDirection);
            }

            robot.setDrivePower(powers[0], powers[1], powers[2], powers[3]);
        }

    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("Controler Directions");
        engine.telemetry.addData("Right", rightJoystickDegrees);
        engine.telemetry.addData("Left", leftJoystickDegrees);

        engine.telemetry.addData("face", faceDirection);

        engine.telemetry.addData("Player 1 children", childrenHaveFinished());
        for (CyberarmState state : children) {
            if (!state.getHasFinished()) {
                engine.telemetry.addLine("" + state.getClass());
            }
        }
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
