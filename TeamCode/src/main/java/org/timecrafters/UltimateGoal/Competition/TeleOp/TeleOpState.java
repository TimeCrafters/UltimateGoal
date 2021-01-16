package org.timecrafters.UltimateGoal.Competition.TeleOp;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Autonomous.DriveToCoordinates;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class TeleOpState extends CyberarmState {

    private Robot robot;
    private float leftJoystickDegrees;
    private double leftJoystickMagnitude;
    private float rightJoystickDegrees;
    private double rightJoystickMagnitude;

    private double POWER_SPRINT = 0.4;
    private double POWER_NORMAL = 0.2;
    private double powerScale = 0.2 ;
    private boolean toggleSpeedInput = false;


    private boolean launchInput = false;
    private CyberarmState launchDriveState = new DriveToCoordinates(robot.LAUNCH_POSITION_X, robot.LAUNCH_POSITION_Y, robot.LAUNCH_ROTATION, 10, POWER_NORMAL, 50);


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

        double[] powers = {0,0,0,0};

        if (engine.gamepad1.y) {
            double distanceToTarget = Math.hypot(robot.LAUNCH_POSITION_X - robot.getLocationX(), robot.LAUNCH_POSITION_Y - robot.getLocationY());
            if (distanceToTarget > robot.LAUNCH_TOLERANCE_POS) {
                powers = robot.getMecanumPowers(robot.getAngleToPosition(robot.LAUNCH_POSITION_X, robot.LAUNCH_POSITION_Y), POWER_NORMAL, robot.LAUNCH_ROTATION);

            } else if (robot.getRelativeAngle(robot.LAUNCH_ROTATION, robot.getRotation()) > robot.LAUNCH_TOLERANCE_FACE) {
                //todo add launch sequence

            }
        } else {
            boolean joystickButton = engine.gamepad1.left_stick_button;

            if (joystickButton && !toggleSpeedInput) {
                if (powerScale == POWER_NORMAL) {
                    powerScale = POWER_SPRINT;
                } else {
                    powerScale = POWER_NORMAL;
                }
            }

            toggleSpeedInput = joystickButton;


            if (rightJoystickMagnitude == 0) {
                if (leftJoystickMagnitude !=0) {
                    powers = robot.getMecanumPowers(leftJoystickDegrees, powerScale * leftJoystickMagnitude, leftJoystickDegrees);
                }
            } else {
                if (leftJoystickMagnitude == 0) {
                    double[] facePowers =  robot.getFacePowers(rightJoystickDegrees, powerScale * rightJoystickMagnitude);
                    powers = new double[]{facePowers[0], facePowers[1], facePowers[0], facePowers[1]};
                } else {
                    powers = robot.getMecanumPowers(leftJoystickDegrees, powerScale * leftJoystickMagnitude, rightJoystickDegrees);
                }
            }
        }

        robot.setDrivePower(powers[0],powers[1],powers[2],powers[3]);

    }
}
