package org.timecrafters.UltimateGoal.Calibration;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Engagable;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.DriveMotor;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class MotorCheck extends CyberarmState {

    private Robot robot;
    private double[] Powers;
    private DcMotor[] Motors;
    private int currentMotor = 0;
    private double currentPower = 0.001;

    private boolean yPrevious;


    public MotorCheck(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        Motors = new DcMotor[]{robot.driveFrontLeft, robot.driveFrontRight, robot.driveBackLeft, robot.driveBackRight};
    }

    @Override
    public void start() {
        Motors[currentMotor].setPower(0.2);
    }

    @Override
    public void exec() {

        if (currentMotor <= 3) {

            boolean y = engine.gamepad1.y;
            if (y && !yPrevious) {

                Motors[currentMotor].setPower(0);
                currentMotor += 1;
                Motors[currentMotor].setPower(0.2);
                sleep(20);
            }
            yPrevious = y;
        }

    }

    @Override
    public void telemetry() {

        engine.telemetry.addData("current Motor", Motors[currentMotor].toString());

    }
}
