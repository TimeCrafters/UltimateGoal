package org.timecrafters.UltimateGoal.Competition.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class FindWobbleGoal extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    private double power;
    private double turnCheck;
    private double driveCheck;
    private double range;
    private boolean cCheckPrev;
    private boolean ccCheckPrev;

    public FindWobbleGoal(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    @Override
    public void init() {
        power = robot.stateConfiguration.variable(groupName,actionName,"power").value();
        turnCheck = robot.stateConfiguration.variable(groupName,actionName,"max").value();
        driveCheck = robot.stateConfiguration.variable(groupName,actionName,"min").value();
        range = robot.stateConfiguration.variable(groupName,actionName,"r").value();
    }

    @Override
    public void start() {
        setHasFinished(!robot.stateConfiguration.action(groupName,actionName).enabled);
        robot.setDrivePower(power,-power,power,-power);
    }

    @Override
    public void exec() {
        robot.updateLocation();
        double sensorValue = robot.wobbleColorSensor.getDistance(DistanceUnit.MM);

        if (sensorValue > turnCheck) {
            boolean cCheck = robot.getRotation() > range;
            boolean ccCheck = robot.getRotation() < -range;

            if (cCheck && !cCheckPrev) {
                robot.setDrivePower(-power, power, -power, power);
            }

            if (ccCheck && !ccCheckPrev) {
                robot.setDrivePower(power, -power, power, -power);
            }
            cCheckPrev = cCheck;
            ccCheckPrev = ccCheck;

        } else {
             if (sensorValue > driveCheck) {
                 robot.setDrivePower(-power,-power,-power,-power);
             } else {
                 robot.setDrivePower(0,0,0,0);
                 setHasFinished(true);
             }
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("sensor", robot.wobbleColorSensor.getDistance(DistanceUnit.MM));
        engine.telemetry.addData("red", robot.wobbleColorSensor.red());
        engine.telemetry.addData("green", robot.wobbleColorSensor.green());
        engine.telemetry.addData("blue", robot.wobbleColorSensor.blue());
    }
}

