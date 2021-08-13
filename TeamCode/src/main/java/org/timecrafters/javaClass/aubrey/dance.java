package org.timecrafters.javaClass.aubrey;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class dance extends CyberarmState {

    private SampleRobot robot;
    private double speed =0.5;
    private  float RotationLimit =20;
    private  double powerWorks =0.5;

    public dance(SampleRobot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        float rotation = robot.getIMURotation();

        robot.driveBackRight.setPower(powerWorks);
        robot.driveBackLeft.setPower(-powerWorks);
    robot.driveFrontLeft.setPower(-powerWorks);
    robot.driveFrontRight.setPower(powerWorks);

    if (-1 > rotation && rotation> -15){
        powerWorks = 0;

    }
robot. ledDriver. setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
    }

    @Override
    public void telemetry() {engine.telemetry.addData("rotation",robot.getIMURotation());
    }
}
