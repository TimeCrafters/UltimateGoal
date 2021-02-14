package org.timecrafters.UltimateGoal.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Launch;
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

    private float scoreAFaceAngle;
    private double scoreATolerance;
    private double scoreAPower;
    private long scoreABrakeTime;

    float scoreBFaceAngle;
    double scoreBTolerance;
    double scoreBPower;
    long scoreBBrakeTime;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        robot.webCamServo.setPosition(Robot.CAM_SERVO_DOWN);

        launchTolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto","04_0","tolPos").value());
        launchPower = robot.stateConfiguration.variable("auto","04_0","power").value();
        launchBrakeTime = robot.stateConfiguration.variable("auto","04_0","brakeMS").value();

//        scoreAFaceAngle = robot.stateConfiguration.variable("auto","05_0","face").value();
//        scoreATolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto","05_0","tolPos").value());
//        scoreAPower = robot.stateConfiguration.variable("auto","05_0","power").value();
//        scoreABrakeTime = robot.stateConfiguration.variable("auto","05_0","brakeMS").value();
//
//        scoreBFaceAngle = robot.stateConfiguration.variable("auto","10_0","face").value();
//        scoreBTolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto","10_0","tolPos").value());
//        scoreBPower = robot.stateConfiguration.variable("auto","10_0","power").value();
//        scoreBBrakeTime = robot.stateConfiguration.variable("auto","10_0","brakeMS").value();

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

        //
        addState(new Face(robot, "auto", "04_1"));

        //launch rings
        addState(new Launch(robot, "auto", "04_2"));

        //drive to scoring area
        addState(new DriveToCoordinates(robot, "auto", "05_0", true));
//        addState(new DriveToCoordinates(robot, tensorFlowCheck.wobblePosX,tensorFlowCheck.wobblePosY,scoreAFaceAngle,scoreATolerance,scoreAPower,scoreABrakeTime));

        //turn arm towards scoreing area.
        ArrayList<CyberarmState> threadStates = new ArrayList<>();
         threadStates.add(new Face(robot, "auto", "05_1"));
        threadStates.add(new WobbleArm(robot, "auto", "05_2",false));
        addState(new ThreadStates(threadStates));

        //open the wobble grab arm
        addState(new WobbleGrab(robot, "auto", "06_0", true));

        //drive to second wobble
        addState(new DriveToCoordinates(robot, "auto","06_1"));

        addState(new DriveToCoordinates(robot, "auto", "07_0"));
        addState(new DriveWithColorSensor(robot, "auto", "08_0"));

        //close grabber
        addState(new WobbleGrab(robot, "auto", "09_0", false));

        //drive to scoring area
//        addState(new DriveToCoordinates(robot, tensorFlowCheck.wobblePosX,tensorFlowCheck.wobblePosY,scoreBFaceAngle,scoreBTolerance,scoreBPower,scoreBBrakeTime));
        addState(new DriveToCoordinates(robot, "auto", "10_0", true));

        //release wobble goal 2
        addState(new Face(robot, "auto", "11_0"));
        addState(new WobbleGrab(robot, "auto", "12_0", true));

        //drive to park
        ArrayList<CyberarmState> threadStatesB = new ArrayList<>();
        threadStatesB.add(new WobbleGrab(robot, "auto", "12_1", false));
        threadStatesB.add(new WobbleArm(robot, "auto", "12_2",true));
        threadStatesB.add(new DriveToCoordinates(robot, "auto", "13_0"));
        addState(new ThreadStates(threadStatesB));
    }
}
