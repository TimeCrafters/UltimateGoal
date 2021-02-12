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

    public DriveToCoordinates(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
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
            xTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "xPos").value());
            yTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "yPos").value());
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

        if (robot.stateConfiguration.action(groupName,actionName).enabled) {
            setHasFinished(true);
        }

        if (autoFace) {
            faceAngle = robot.getAngleToPosition(xTarget,yTarget);
        }
    }

    @Override
    public void exec() {

        double distanceToTarget = Math.hypot(xTarget - robot.getLocationX(), yTarget - robot.getLocationY());

        if (distanceToTarget > tolerancePos) {
            double[] powers = robot.getMecanumPowers(robot.getAngleToPosition(xTarget,yTarget), power, faceAngle);
            robot.setDrivePower(powers[0], powers[1],powers[2],powers[3]);
            braking = false;
        } else {
            long currentTime = System.currentTimeMillis();
            if (!braking) {
                breakStartTime = currentTime;
                braking = true;
            }
            setHasFinished(currentTime - breakStartTime >= brakeTime);
        }

    }
}
