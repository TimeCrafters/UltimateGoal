package org.timecrafters.UltimateGoal.Calibration;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.DriveMotor;
import org.timecrafters.UltimateGoal.Robot;

public class VelocityTest extends CyberarmState {

    private Robot robot;
    private double[] Velocities;
    private DriveMotor[] Motors;
    private long LastTime = 0;



    public  VelocityTest(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        Motors = new DriveMotor[]{robot.driveFrontLeft, robot.driveFrontRight, robot.driveBackLeft, robot.driveBackRight};
        Velocities = new double[] {0,0,0,0};

    }

    @Override
    public void start() {
        for (DriveMotor motor : Motors) {
            motor.setPower(0.1);
        }
    }

    @Override
    public void exec() {

        for (int i = 0; i < 4; i++) {
            Velocities[i] = (Motors[i].motor.getCurrentPosition()* 60000) / runTime();
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("FrontLeft", Velocities[0]);
        engine.telemetry.addData("FrontRight", Velocities[1]);
        engine.telemetry.addData("BackLeft", Velocities[2]);
        engine.telemetry.addData("BackRight", Velocities[3]);
    }
}
