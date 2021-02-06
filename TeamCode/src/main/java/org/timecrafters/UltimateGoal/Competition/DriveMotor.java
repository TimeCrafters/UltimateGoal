package org.timecrafters.UltimateGoal.Competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DriveMotor {

    public DcMotor motor;
    private int brakePosition;
    private MotorConfigurationType motorType;
    private DcMotorSimple.Direction direction;
    static final double FINE_CORRECTION = 0.05;
    static final double LARGE_CORRECTION = 0.03;

    public DriveMotor(DcMotor motor, MotorConfigurationType motorType, DcMotorSimple.Direction direction) {
        this.motor = motor;
        this.motorType = motorType;
        this.direction = direction;
    }

    public void init() {
//        motor.setMotorType(motorType);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(direction);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setBrakePosition() {
        brakePosition = motor.getCurrentPosition();
    }

    public void brake() {
        double currentPosition = motor.getCurrentPosition();
        double relativePosition = brakePosition - currentPosition;
        double breakPower = Math.pow(LARGE_CORRECTION * relativePosition, 3) + FINE_CORRECTION * relativePosition;


    }

}
