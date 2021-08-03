package org.timecrafters.UltimateGoal.Competition.TeleOp;

/*
The Player1 state has all the controls for player one. The Player One and Player Two controls are
kept in separate states so that the childrenHaveFinished() method can be used to easily stop things
from running at the same time that shouldn't be.
*/

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Autonomous.DriveToCoordinates;
import org.timecrafters.UltimateGoal.Competition.Autonomous.FindWobbleGoal;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class Player1 extends CyberarmState {
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
    private boolean aPrev;

    //Drive to launch control
    private powerShotsControl powerShots;
    private boolean runNextDriveToLaunch;
    private boolean driveToLaunchInputPrev;

    private double endGameX;
    private double endGameY;
    private float endGameRot;
    private int loopCount;

    private double refinePower;

    public Player1(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        cardinalSnapping = robot.stateConfiguration.variable(
                "tele","control", "cardinalSnapping").value();
        pairSnapping = robot.stateConfiguration.variable(
                "tele","control", "pairSnapping").value();
        faceControlThreshold = robot.stateConfiguration.variable(
                "tele","control", "faceControlT").value();
        refinePower = robot.stateConfiguration.variable(
                "tele","control", "refPower").value();

        endGameX = robot.stateConfiguration.variable(
                "tele","_endGameStart","x").value();
        endGameY = robot.stateConfiguration.variable(
                "tele","_endGameStart","y").value();
        endGameRot = robot.stateConfiguration.variable(
                "tele","_endGameStart", "r").value();
    }

    @Override
    public void start() {
        faceDirection = robot.getRotation();
    }

    @Override
    public void exec() {
        loopCount+=1;
        robot.record(""+runTime()+", "+loopCount+", "+robot.getRotation());
        robot.updateLocation();

        //toggle for drive speed
        boolean lb = engine.gamepad1.left_stick_button;
        if (lb && !lbPrev) {
            if (drivePower == 1) {
                drivePower = 0.5;
            } else {
                drivePower = 1;
            }
        }
        lbPrev = lb;

        //This Runs the FindWobbleGoal state as long as the button is pressed. When it's released,
        //the state is interrupted. If the state finishes while the button is still pressed,
        //it waits until the button is released before running the state again
        runNextFindWobble = (findWobbleGoal == null || findWobbleGoal.getHasFinished());

        boolean findWobbleInput = engine.gamepad1.x;
        if (findWobbleInput) {
            //if the claw is open, run FindWobbleGoal
            if (robot.wobbleGrabServo.getPosition() == Robot.WOBBLE_SERVO_OPEN) {

                faceDirection = robot.getRotation();

                if (runNextFindWobble && !findWobbleInputPrev) {
                    findWobbleGoal =
                            new FindWobbleGoal(robot, "auto", "08_0");
                    addParallelState(findWobbleGoal);
                }
                //if the claw is closed, open the claw.
            } else if (!findWobbleInputPrev) {
                robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_OPEN);
            }
            //if the button is released cancel the search
        } else if (!runNextFindWobble) {
            findWobbleGoal.setHasFinished(true);
        }
        findWobbleInputPrev = findWobbleInput;

        //toggles wobble grabber open and closed
        boolean a = engine.gamepad1.a;
        if (a && !aPrev) {
            if (robot.wobbleGrabServo.getPosition() == Robot.WOBBLE_SERVO_OPEN) {
                robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_CLOSED);
            } else {
                robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_OPEN);
            }
        }
        aPrev = a;

        //This logic works the same as the stuff above, accept instead of autonomous wobble grab,
        //it
        runNextDriveToLaunch = (powerShots == null || powerShots.getHasFinished());

        boolean driveToLaunchInput = engine.gamepad1.y && !findWobbleInput;
        if (driveToLaunchInput) {
            if (runNextDriveToLaunch && !driveToLaunchInputPrev) {
                powerShots = new powerShotsControl(robot);
                addParallelState(powerShots);
            }
            faceDirection = robot.getRotation();
        } else if (!runNextDriveToLaunch) {
            powerShots.setHasFinished(true);
        }
        driveToLaunchInputPrev = driveToLaunchInput;

        //Normal Driver Controls
        if (childrenHaveFinished()) {


            double leftJoystickX = engine.gamepad1.left_stick_x;
            double leftJoystickY = engine.gamepad1.left_stick_y;

            leftJoystickDegrees = robot.getRelativeAngle(90,
                    (float) Math.toDegrees(Math.atan2(leftJoystickX, -leftJoystickY)));
            leftJoystickMagnitude = Math.hypot(leftJoystickX, leftJoystickY);

            double rightJoystickX = engine.gamepad1.right_stick_x;
            double rightJoystickY = engine.gamepad1.right_stick_y;

            rightJoystickDegrees = robot.getRelativeAngle(90,
                    (float) Math.toDegrees(Math.atan2(rightJoystickX, -rightJoystickY)));
            rightJoystickMagnitude = Math.hypot(rightJoystickX, rightJoystickY);

            //allows the the driver to indicate which direction the robot is currently looking
            //so that the controller and robot can be re-synced in the event of a bad initial
            //rotation.
            if (engine.gamepad1.back) {
                float newRotation = 0;

                if (rightJoystickMagnitude != 0) {
                    newRotation = rightJoystickDegrees;
                }

                robot.setLocalization(newRotation, robot.getLocationX(), robot.getLocationY());
                faceDirection = newRotation;
            }

            //if the driver is letting go of the face joystick, the robot should maintain it's
            //current face direction.
            if (rightJoystickMagnitude > faceControlThreshold ||
                    rightJoystickMagnitude - rightJoystickMagnitudePrevious > 0) {

                //if the joystick is close to one of the cardinal directions, it is set to exactly
                // the cardinal direction
                faceDirection = snapToCardinal(rightJoystickDegrees, cardinalSnapping, 0);
            }
            rightJoystickMagnitudePrevious = rightJoystickMagnitude;

            //The D-pad is used if the drivers need to get a more precise angle than they can get
            //using the face joystick
            if (engine.gamepad1.dpad_right) {
                powers = new double[]{refinePower, -refinePower, refinePower, -refinePower};
                faceDirection = robot.getRotation();
            } else if (engine.gamepad1.dpad_left) {
                powers = new double[]{-refinePower, refinePower, -refinePower, refinePower};
                faceDirection = robot.getRotation();
            } else if (leftJoystickMagnitude == 0) {
                double[] facePowers = robot.getFacePowers(faceDirection, 0.4);
                powers = new double[]{facePowers[0], facePowers[1], facePowers[0], facePowers[1]};
            } else {
                //drives the robot in the direction of the move joystick while facing the direction
                //of the look joystick. if the move direction is almost aligned/perpendicular to the
                //look joystick,
                powers = robot.getMecanumPowers(
                        snapToCardinal(leftJoystickDegrees, pairSnapping, faceDirection),
                        drivePower, faceDirection);
            }

            robot.setDrivePower(powers[0], powers[1], powers[2], powers[3]);
        }

        //LED feedback control
        double ringBeltPower = robot.ringBeltMotor.getPower();
        if (ringBeltPower > 0 && Math.abs(robot.ringBeltMotor.getTargetPosition() -
                robot.ringBeltMotor.getCurrentPosition()) > 10) {
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE );
        } else if (ringBeltPower < 0) {
            robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        }  else {
            if (drivePower == 1) {
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
            } else {
                robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            }
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
