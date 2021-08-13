package org.timecrafters.javaClass.cayden;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class Cayden_Autonomous extends CyberarmState {
    private Robot robot;
    private double variable=.5;
    private double inches=12;

    public Cayden_Autonomous(Robot robot, double variable, double inches) {
        this.robot = robot;
        this.variable = variable;
        this.inches = inches;
    }

    @Override
    public void exec() {

        robot.driveFrontRight.setPower(variable);
        robot.driveBackRight.setPower(variable);
        robot.driveFrontLeft.setPower(variable);
        robot.driveBackLeft.setPower(variable);


        if (Math.abs(robot.encoderLeft.getCurrentPosition())>=robot.inchesToTicks(inches)){
            variable=0;
        }
robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Encoder", robot.encoderLeft.getCurrentPosition());
    }
}
