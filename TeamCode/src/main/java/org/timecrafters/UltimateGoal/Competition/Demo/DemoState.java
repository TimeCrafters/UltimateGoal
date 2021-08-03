package org.timecrafters.UltimateGoal.Competition.Demo;

/*
The Player1 state has all the controls for player one. The Player One and Player Two controls are
kept in separate states so that the childrenHaveFinished() method can be used to easily stop things
from running at the same time that shouldn't be.
*/

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Autonomous.FindWobbleGoal;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.UltimateGoal.Competition.TeleOp.powerShotsControl;

public class DemoState extends CyberarmState {
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

    private double refinePower;

    public DemoState(Robot robot) {
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

    }

    @Override
    public void start() {
        faceDirection = robot.getRotation();
    }

    @Override
    public void exec() {
        robot.updateLocation();

        Gamepad gamepad = engine.gamepad1;
        if (engine.gamepad2.right_trigger != 0) {
            gamepad = engine.gamepad2;
        }

        //toggle for drive speed
        boolean lb = engine.gamepad2.left_stick_button;
        if (lb && !lbPrev) {
            if (drivePower == 1) {
                drivePower = 0.5;
            } else {
                drivePower = 1;
            }
        }
        lbPrev = lb;

        //Calculate Joystick Vector
        double leftJoystickX = gamepad.left_stick_x;
        double leftJoystickY = gamepad.left_stick_y;

        leftJoystickDegrees = robot.getRelativeAngle(0,
                (float) Math.toDegrees(Math.atan2(leftJoystickX, -leftJoystickY)));
        leftJoystickMagnitude = Math.hypot(leftJoystickX, leftJoystickY);

        double rightJoystickX = gamepad.right_stick_x;
        double rightJoystickY = gamepad.right_stick_y;

        rightJoystickDegrees = robot.getRelativeAngle(0,
                (float) Math.toDegrees(Math.atan2(rightJoystickX, -rightJoystickY)));
        rightJoystickMagnitude = Math.hypot(rightJoystickX, rightJoystickY);

        //allows the the driver to indicate which direction the robot is currently looking
        //so that the controller and robot can be re-synced in the event of a bad initial
        //rotation.
        if (gamepad.back) {
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
        if (gamepad.dpad_right) {
            powers = new double[]{refinePower, -refinePower, refinePower, -refinePower};
            faceDirection = robot.getRotation();
        } else if (gamepad.dpad_left) {
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

    //This just checks if the wobble arm runmode is already the desired mode before setting it.
    //I don't know if this is actually helpful
    private void setArmMode(DcMotor.RunMode runMode) {
        if (robot.wobbleArmMotor.getMode() != runMode) {
            robot.wobbleArmMotor.setMode(runMode);
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
