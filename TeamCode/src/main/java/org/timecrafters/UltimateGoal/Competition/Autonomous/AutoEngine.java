package org.timecrafters.UltimateGoal.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Launch;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.UltimateGoal.Competition.WobbleGrab;

@Autonomous (name = "Autonomous")
public class AutoEngine extends CyberarmEngine {

    private Robot robot;
    private TensorFlowCheck tensorFlowCheck;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.initHardware();
        super.init();
    }

    @Override
    public void setup() {
        //drive to view
        addState(new DriveToCoordinates(robot, "auto", "01_0"));

        //select scoring area
        tensorFlowCheck = new TensorFlowCheck(robot, "auto", "02_0");
        addState(tensorFlowCheck);

        //drive around ring stack
        addState(new DriveToCoordinates(robot, "auto", "03_0"));

        //drive to launch position
        double launchTolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto","04_0","tolPos").value());
        double launchPower = robot.stateConfiguration.variable("auto","04_0","power").value();
        long launchBrakeTime = robot.stateConfiguration.variable("auto","04_0","bakeMS").value();
        addState(new DriveToCoordinates(robot, robot.launchPositionX,robot.launchPositionY,robot.launchRotation,launchTolerance,launchPower,launchBrakeTime));

        //launch rings
        addState(new Launch(robot));

        //drive to scoring area
        float scoreAFaceAngle = robot.stateConfiguration.variable("auto","05_0","faceAngle").value();
        double scoreATolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto","05_0","tolPos").value());
        double scoreAPower = robot.stateConfiguration.variable("auto","05_0","power").value();
        long scoreABrakeTime = robot.stateConfiguration.variable("auto","05_0","bakeMS").value();
        addState(new DriveToCoordinates(robot, tensorFlowCheck.wobblePosX,tensorFlowCheck.wobblePosY,scoreAFaceAngle,scoreATolerance,scoreAPower,scoreABrakeTime));

        //turn arm towards scoreing area.
        addState(new Face(robot, "auto", "05_1"));

        //open the wobble grab arm
        addState(new WobbleGrab(robot, "auto", "06_0", true));

        //drive to second wobble
        addState(new DriveToCoordinates(robot, "auto", "07_0"));
        addState(new DriveWithColorSensor(robot, "auto", "08_0"));

        //close grabber
        addState(new WobbleGrab(robot, "auto", "09_0", false));

        //drive to scoring area
        float scoreBFaceAngle = robot.stateConfiguration.variable("auto","05_0","faceAngle").value();
        double scoreBTolerance = robot.inchesToTicks((double) robot.stateConfiguration.variable("auto","05_0","tolPos").value());
        double scoreBPower = robot.stateConfiguration.variable("auto","05_0","power").value();
        long scoreBBrakeTime = robot.stateConfiguration.variable("auto","05_0","bakeMS").value();
        addState(new DriveToCoordinates(robot, tensorFlowCheck.wobblePosX,tensorFlowCheck.wobblePosY,scoreBFaceAngle,scoreBTolerance,scoreBPower,scoreBBrakeTime));

        //release wobble goal 2
        addState(new WobbleGrab(robot, "auto", "10_0", true));

        //drive to park
        addState(new DriveToCoordinates(robot, "auto", "11_0"));
    }
}
