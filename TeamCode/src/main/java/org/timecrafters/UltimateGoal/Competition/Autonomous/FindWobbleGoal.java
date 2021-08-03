package org.timecrafters.UltimateGoal.Competition.Autonomous;

/*
The FindWobbleGoal state is used in teleOp and Autonomous to aid in capturing the wobble goal.
*/

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
    private float startRotation;
    private boolean foundGoalRotation;
    private float wobbleGoalRotation;
    private long timeLimit;

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
        timeLimit = robot.stateConfiguration.variable(groupName,actionName,"limit").value();
    }

    @Override
    public void start() {
        setHasFinished(!robot.stateConfiguration.action(groupName,actionName).enabled);
        startRotation = robot.getRotation();
        robot.setDrivePower(power,-power,power,-power);
    }

    @Override
    public void exec() {
        robot.updateLocation();
        double sensorValue = robot.wobbleColorSensor.getDistance(DistanceUnit.MM);

        //Stage 1: scan back and forth untile the sensor is in line with the wobble goal.
        if (sensorValue > turnCheck) {

            float rotation = robot.getRelativeAngle(startRotation,robot.getRotation());

            boolean cCheck = rotation > range;
            boolean ccCheck = rotation < -range;

            if (cCheck && !cCheckPrev) {
                robot.setDrivePower(-power, power, -power, power);
            }

            if (ccCheck && !ccCheckPrev) {
                robot.setDrivePower(power, -power, power, -power);
            }
            cCheckPrev = cCheck;
            ccCheckPrev = ccCheck;

        } else {
            //Stage 2: drive toward wobble goal until it's close enough to grab
             if (sensorValue > driveCheck) {
                 if (!foundGoalRotation) {
                     foundGoalRotation = true;
                     wobbleGoalRotation = robot.getRotation();
                 }
                 double[] powers = robot.getMecanumPowers(
                         wobbleGoalRotation - 90,
                         power*2 , wobbleGoalRotation);
                 robot.setDrivePower(powers[0],powers[1],powers[2],powers[3]);
             } else {
                 //stage 3: grab the wobble goal && finish the state
                 endSearch();
             }
        }

        //if the search takes too long, the robot grabs and finishes the state
        if (runTime() > timeLimit) {
            endSearch();
        }
    }

    private void endSearch() {
        robot.setDrivePower(0,0,0,0);
        robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_CLOSED);
        setHasFinished(true);
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("sensor", robot.wobbleColorSensor.getDistance(DistanceUnit.MM));
        engine.telemetry.addData("red", robot.wobbleColorSensor.red());
        engine.telemetry.addData("green", robot.wobbleColorSensor.green());
        engine.telemetry.addData("blue", robot.wobbleColorSensor.blue());
    }
}

