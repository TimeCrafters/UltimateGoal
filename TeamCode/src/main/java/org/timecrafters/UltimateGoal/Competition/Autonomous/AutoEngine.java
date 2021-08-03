package org.timecrafters.UltimateGoal.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Launch;
import org.timecrafters.UltimateGoal.Competition.Pause;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.UltimateGoal.Competition.WobbleArm;
import org.timecrafters.UltimateGoal.Competition.WobbleGrab;

import java.util.ArrayList;

@Autonomous (name = "Autonomous")
public class AutoEngine extends CyberarmEngine {

    private Robot robot;
    private TensorFlowCheck tensorFlowCheck;

    private double launchTolerance;
    private double launchPower;
    private long launchBrakeTime;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        robot.webCamServo.setPosition(Robot.CAM_SERVO_DOWN);
        robot.wobbleGrabServo.setPosition(Robot.WOBBLE_SERVO_CLOSED);
        robot.wobbleArmMotor.setTargetPosition(0);
        robot.wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // since we've preloaded three rings, the ring belt stage is set to account for this;
        robot.ringBeltStage = 3;

        //Autonomous specific Configuration Variables
        float rotation = robot.stateConfiguration.variable("system", "startPos", "direction").value();
        double locationX = robot.inchesToTicks((double) robot.stateConfiguration.variable("system", "startPos", "x").value());
        double locationY = robot.inchesToTicks((double) robot.stateConfiguration.variable("system", "startPos", "y").value());
        robot.setLocalization(rotation,locationX,locationY);

        launchTolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto","04_0","tolPos").value());
        launchPower = robot.stateConfiguration.variable("auto","04_0","power").value();
        launchBrakeTime = robot.stateConfiguration.variable("auto","04_0","brakeMS").value();

        super.init();
    }

    @Override
    public void setup() {
        //drive to view
        addState(new DriveToCoordinates(robot, "auto", "01_0"));

        //face ring
        addState(new Face(robot, "auto", "01_1"));

        //select scoring area
        tensorFlowCheck = new TensorFlowCheck(robot, "auto", "02_0");
        addState(tensorFlowCheck);

        //drive around ring stack
        addState(new DriveToCoordinates(robot, "auto", "03_0"));

        //drive to launch position
        addState(new DriveToCoordinates(robot, robot.launchPositionX,robot.launchPositionY,robot.launchRotation,launchTolerance,launchPower,launchBrakeTime));

        //aligns to goal
        addState(new Face(robot, "auto", "04_1"));

        //launch rings and drive to scoreing area. LaunchDriveControl allows makes the robot begin
        //driving while the belt is resetting
        CyberarmState launchState = new Launch(robot, "auto", "04_2");

        CyberarmState driveState = new DriveToCoordinates(robot, "auto", "05_0", true);
        addState(new LaunchDriveControl(robot,launchState,driveState));

        //turn arm towards scoreing area.
        ArrayList<CyberarmState> threadStates0 = new ArrayList<>();
         threadStates0.add(new Face(robot, "auto", "05_1"));
        threadStates0.add(new WobbleArm(robot, "auto", "05_2",robot.wobbleDownPos));
        addState(new ThreadStates(threadStates0));

        //open the wobble grab arm
        addState(new WobbleGrab(robot, "auto", "06_0", true));

        //drive to second wobble
        addState(new DriveToCoordinates(robot, "auto","06_1"));

        addState(new DriveToCoordinates(robot, "auto", "07_0"));
        addState(new Face(robot,"auto","07_1"));

        addState(new FindWobbleGoal(robot, "auto", "08_0"));
        addState(new Pause(robot,"auto","09_0"));

        //drive to scoring area
        ArrayList<CyberarmState> threadStates1 = new ArrayList<>();
//        threadStates1.add(new Face(robot, "auto", "09_0"));
        threadStates1.add(new WobbleArm(robot, "auto", "09_1",robot.wobbleUpPos));
        threadStates1.add(new DriveToCoordinates(robot, "auto", "10_0", true));
        addState(new ThreadStates(threadStates1));



        ArrayList<CyberarmState> threadStates2 = new ArrayList<>();
        threadStates2.add(new Face(robot, "auto", "11_0"));
        threadStates2.add(new WobbleArm(robot, "auto", "11_1",robot.wobbleDownPos));
        addState(new ThreadStates(threadStates2));

        //release wobble goal 2
        addState(new WobbleGrab(robot, "auto", "12_0", true));

        //drive to park
        ArrayList<CyberarmState> threadStates3 = new ArrayList<>();
        threadStates3.add(new WobbleArm(robot, "auto", "12_2", 0));
        threadStates3.add(new Park(robot,"auto","13_0"));
        addState(new ThreadStates(threadStates3));
    }
}
