package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class DriveWithColorSensor extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    private double power;
    private double minimum;
    private double maximum;

    public DriveWithColorSensor(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    @Override
    public void init() {
        power = robot.stateConfiguration.variable(groupName,actionName,"power").value();
        minimum = robot.stateConfiguration.variable(groupName,actionName,"min").value();
        maximum = robot.stateConfiguration.variable(groupName,actionName,"max").value();
    }

    @Override
    public void start() {
        setHasFinished(!robot.stateConfiguration.action(groupName,actionName).enabled);
    }

    @Override
    public void exec() {
        double sensorValue = robot.wobbleColorSensor.getDistance(DistanceUnit.MM);
        if (sensorValue < minimum) {
            robot.getMecanumPowers(180,power,0);
        } else if (sensorValue > maximum) {
            robot.getMecanumPowers(0,power,0);
        } else {
            robot.setDrivePower(0,0,0,0);
            setHasFinished(true);
        }
    }
}
