package org.timecrafters.UltimateGoal.Calibration;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;

import java.util.ArrayList;

public class StalPowerCalibrator extends CyberarmState {

    private Robot robot;
    private double[] Powers;
    private DcMotor[] Motors;
    private int currentMotor = 0;
    private double currentPower = 0.001;



    public StalPowerCalibrator(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        Motors = new DcMotor[]{robot.driveFrontLeft, robot.driveFrontRight, robot.driveBackLeft, robot.driveBackRight};
        Powers = new double[] {0,0,0,0};

        for (DcMotor motor : Motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void exec() {

        if (currentMotor <= 3) {



            if (Math.abs(Motors[currentMotor].getCurrentPosition()) < 10 && currentPower < 1) {
                currentPower += 0.001;
                Motors[currentMotor].setPower(currentPower);
                sleep(20);
            } else {
                Powers[currentMotor] = currentPower;
                Motors[currentMotor].setPower(0);
                currentPower = 0.001;
                currentMotor += 1;
                sleep(1000);
                for (DcMotor motor : Motors) {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
        }

    }

    @Override
    public void telemetry() {

        engine.telemetry.addData("current Power", currentPower);
        engine.telemetry.addData("current Motor", currentMotor);

        engine.telemetry.addData("FrontLeft", Powers[0]);
        engine.telemetry.addData("FrontRight", Powers[1]);
        engine.telemetry.addData("BackLeft", Powers[2]);
        engine.telemetry.addData("BackRight", Powers[3]);
    }
}
