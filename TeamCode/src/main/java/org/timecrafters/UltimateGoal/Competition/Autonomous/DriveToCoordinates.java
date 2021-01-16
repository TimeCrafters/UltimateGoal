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
    private double tolerance;
    private double power;
    private boolean braking;
    private long breakStartTime;
    private long breakTime;

    public DriveToCoordinates(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    public DriveToCoordinates(double xTarget, double yTarget, float faceAngle, double tolerance, double power, long breakStartTime) {
        this.xTarget = xTarget;
        this.yTarget = yTarget;
        this.faceAngle = faceAngle;
        this.tolerance = tolerance;
        this.power = power;
        this.breakTime = breakTime;
    }

    @Override
    public void init() {
        if (!groupName.equals("manual")) {
            xTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "x").value());
            yTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "y").value());
            power = robot.stateConfiguration.variable(groupName, actionName, "power").value();
            faceAngle = robot.stateConfiguration.variable(groupName, actionName, "face").value();
            tolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "tol").value());
            breakTime = robot.stateConfiguration.variable(groupName, actionName, "breakMS").value();
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void exec() {

        double distanceToTarget = Math.hypot(xTarget - robot.getLocationX(), yTarget - robot.getLocationY());

        if (distanceToTarget > tolerance) {
            double[] powers = robot.getMecanumPowers(robot.getAngleToPosition(xTarget,yTarget), power, faceAngle);
            robot.setDrivePower(powers[0], powers[1],powers[2],powers[3]);
            braking = false;
        } else {
            if (!braking) {
                breakStartTime = System.currentTimeMillis();
                robot.setBrakePosAll();
                braking = true;
            }
            robot.brakeAll();
            setHasFinished(System.currentTimeMillis() - breakStartTime > breakTime);
        }

    }
}
