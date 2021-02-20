package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class Face extends CyberarmState {

    private Robot robot;
    private String groupName = "manual";
    private String actionName;
    private float faceAngle;
    private float toleranceFace;
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

    public Face(Robot robot, float faceAngle, float toleranceFace, double power, long breakTime) {
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
            breakTime = robot.stateConfiguration.variable(groupName, actionName, "brakeMS").value();

            try {
                faceAngle = robot.stateConfiguration.variable(groupName, actionName, "faceAngle").value();
            } catch (RuntimeException e) {
                autoFace = true;
            }
        }
    }

    @Override
    public void start() {

        if (!groupName.equals("manual")) {
            setHasFinished(!robot.stateConfiguration.action(groupName, actionName).enabled);


            if (autoFace) {
                double x = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "faceX").value());
                double y = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "faceY").value());
                faceAngle = robot.getAngleToPosition(x, y);
            }

            if (faceAngle == 360) {
                double x = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "faceX").value());
                double y = robot.inchesToTicks((double) robot.stateConfiguration.variable(groupName, actionName, "faceY").value());
                faceAngle = robot.getRelativeAngle(180,robot.getAngleToPosition(x, y));
            }
        }
    }

    @Override
    public void exec() {
        robot.updateLocation();
        double[] powers = robot.getFacePowers(faceAngle,power);
        robot.setDrivePower(powers[0], powers[1],powers[0],powers[1]);
        if (Math.abs(robot.getRelativeAngle(faceAngle,robot.getRotation())) > toleranceFace) {
            braking = false;
        } else {
            long currentTime = System.currentTimeMillis();
            if (!braking) {
                breakStartTime = currentTime;
                braking = true;
            }
            if (currentTime - breakStartTime >= breakTime) {
                setHasFinished(true);
                robot.setDrivePower(0,0,0,0);
            }
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("target angle", faceAngle);
        engine.telemetry.addData("current angle", robot.getRotation());
    }
}
