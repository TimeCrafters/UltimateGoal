package org.timecrafters.javaClass.samples;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class blankTHing extends CyberarmState {

    private Robot robot;
    private float trigger;
    private int someOtherValue = 100;


    public blankTHing(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        robot.ringBeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ringBeltMotor.setTargetPosition(0);
        robot.ringBeltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void exec() {

        robot.magnetSensor.isPressed();

        robot.ringBeltMotor.setTargetPosition(robot.ringBeltMotor.getCurrentPosition() + someOtherValue);

    }
}
