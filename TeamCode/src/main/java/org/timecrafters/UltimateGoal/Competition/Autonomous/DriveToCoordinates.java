package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class DriveToCoordinates extends CyberarmState {

    private Robot robot;
    private String groupName = "manual";
    private String actionName;
    private double xTarget;
    private double yTarget;
    private float faceAngle;
    private double tolerancePos;
    private double power;
    private boolean braking;
    private long breakStartTime;
    private long brakeTime;
    private boolean autoFace;
    private boolean scoringArea;

    public DriveToCoordinates(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    public DriveToCoordinates(Robot robot, String groupName, String actionName, boolean scoringArea) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
        this.scoringArea = scoringArea;
    }

    public DriveToCoordinates(Robot robot, double xTarget, double yTarget, float faceAngle, double tolerance, double power, long brakeTime) {
        this.robot = robot;
        this.xTarget = xTarget;
        this.yTarget = yTarget;
        this.faceAngle = faceAngle;
        this.tolerancePos = tolerance;
        this.power = power;
        this.brakeTime = brakeTime;
    }

    @Override
    public void init() {
        if (!groupName.equals("manual")) {
            power = robot.stateConfiguration.variable(groupName, actionName, "power").value();
            tolerancePos = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "tolPos").value());
            brakeTime = robot.stateConfiguration.variable(groupName, actionName, "brakeMS").value();

            try {
                faceAngle = robot.stateConfiguration.variable(groupName, actionName, "face").value();
            } catch (NullPointerException e) {
                autoFace = true;
            }
        }
    }

    @Override
    public void start() {
        if (!groupName.equals("manual")) {
            setHasFinished(!robot.stateConfiguration.action(groupName, actionName).enabled);

            if (!scoringArea) {
                xTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "xPos").value());
                yTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "yPos").value());
            } else {
                xTarget = robot.wobbleScoreX;
                yTarget = robot.wobbleScoreY;
            }
        }

        if (autoFace) {
            faceAngle = robot.getAngleToPosition(xTarget,yTarget);
        }
    }

    @Override
    public void exec() {
        robot.updateLocation();

        double distanceToTarget = Math.hypot(xTarget - robot.getLocationX(), yTarget - robot.getLocationY());

        double[] powers = robot.getMecanumPowers(robot.getAngleToPosition(xTarget,yTarget), power, faceAngle);
        robot.setDrivePower(powers[0], powers[1],powers[2],powers[3]);

        if (distanceToTarget > tolerancePos) {
            braking = false;
        } else {
            long currentTime = System.currentTimeMillis();
            if (!braking) {
                breakStartTime = currentTime;
                braking = true;
            }
            if (currentTime - breakStartTime >= brakeTime) {
                robot.setDrivePower(0,0,0,0);
                setHasFinished(true);
            }
        }

    }
}
