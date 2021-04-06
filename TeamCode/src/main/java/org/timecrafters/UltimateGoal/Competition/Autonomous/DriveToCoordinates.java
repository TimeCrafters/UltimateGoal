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
    private double basePower;
    private boolean braking;
    private long breakStartTime;
    private long brakeTime;
    private boolean autoFace;
    private double decelRange;
    private double decelMin;
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
        this.basePower = power;
        this.brakeTime = brakeTime;
    }

    public DriveToCoordinates(Robot robot, double xTarget, double yTarget, float faceAngle, double tolerance, double power, long brakeTime, double  decelRange, double decelMin) {
        this.robot = robot;
        this.xTarget = xTarget;
        this.yTarget = yTarget;
        this.faceAngle = faceAngle;
        this.tolerancePos = tolerance;
        this.basePower = power;
        this.brakeTime = brakeTime;
        this.decelRange = decelRange;
        this.decelMin = decelMin;
    }

    @Override
    public void init() {
        if (!groupName.equals("manual")) {
            xTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "xPos").value());
            yTarget = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "yPos").value());
            basePower = robot.stateConfiguration.variable(groupName, actionName, "power").value();
            tolerancePos = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "tolPos").value());
            brakeTime = robot.stateConfiguration.variable(groupName, actionName, "brakeMS").value();

            try {
                faceAngle = robot.stateConfiguration.variable(groupName, actionName, "face").value();
            } catch (RuntimeException  e) {
                autoFace = true;
            }

            try {
                decelRange = robot.stateConfiguration.variable(groupName, actionName, "decelR").value();
                decelMin = robot.stateConfiguration.variable(groupName, actionName, "decelM").value();
            } catch (RuntimeException e) {
                decelRange = 0;
            }
        }

        if (decelRange > 0) {
            decelRange = robot.inchesToTicks(decelRange);
        }

    }

    @Override
    public void start() {
        if (!groupName.equals("manual")) {
            setHasFinished(!robot.stateConfiguration.action(groupName, actionName).enabled);

            //used to navigate towards the randomly generated scoreing area. the original target
            //becomes an offset of the scoring area position.
            if (scoringArea) {
                xTarget += robot.wobbleScoreX;
                yTarget += robot.wobbleScoreY;
            }

            if (faceAngle == 360) {
                faceAngle = robot.getRelativeAngle(180, robot.getAngleToPosition(xTarget,yTarget));
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

        // deceleration
        double power = basePower;
        if (distanceToTarget < decelRange) {
            power = ((distanceToTarget / decelRange) * (basePower - decelMin)) + decelMin;
        }

        double[] powers = robot.getMecanumPowers(robot.getAngleToPosition(xTarget,yTarget), power, faceAngle);
        robot.setDrivePower(powers[0], powers[1],powers[2],powers[3]);

        //Once the robot is within the acceptable range,
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
