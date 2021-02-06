package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class Face extends CyberarmState {

    private Robot robot;
    private String groupName = "manual";
    private String actionName;
    private float faceAngle;
    private double toleranceFace;
    private double power;
    private boolean braking;
    private long breakStartTime;
    private long breakTime;
    private boolean autoFace;

    public Face(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    public Face(Robot robot, float faceAngle, double toleranceFace, double power, long breakTime) {
        this.robot = robot;
        this.faceAngle = faceAngle;
        this.toleranceFace = toleranceFace;
        this.power = power;
        this.breakTime = breakTime;
    }

    @Override
    public void init() {
        if (!groupName.equals("manual")) {
            power = robot.stateConfiguration.variable(groupName, actionName, "power").value();
            toleranceFace = robot.stateConfiguration.variable(groupName, actionName, "tolFace").value();
            breakTime = robot.stateConfiguration.variable(groupName, actionName, "brake MS").value();

            try {
                faceAngle = robot.stateConfiguration.variable(groupName, actionName, "faceAngle").value();
            } catch (NullPointerException e) {
                autoFace = true;
            }
        }
    }

    @Override
    public void start() {
        if (autoFace) {
            double x = robot.stateConfiguration.variable(groupName, actionName, "faceX").value();
            double y = robot.stateConfiguration.variable(groupName, actionName, "faceY").value();
            faceAngle = robot.getAngleToPosition(x,y);
        }
    }

    @Override
    public void exec() {

        if (Math.abs(robot.getRelativeAngle(faceAngle,robot.getRotation())) > toleranceFace) {
            double[] powers = robot.getFacePowers(faceAngle,power);
            robot.setDrivePower(powers[0], powers[1],powers[0],powers[1]);
            braking = false;
        } else {
            long currentTime = System.currentTimeMillis();
            if (!braking) {
                breakStartTime = currentTime;
                braking = true;
            }
            setHasFinished(currentTime - breakStartTime >= breakTime);
        }
    }
}
