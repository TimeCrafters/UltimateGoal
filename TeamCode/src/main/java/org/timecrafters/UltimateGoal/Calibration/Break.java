package org.timecrafters.UltimateGoal.Calibration;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.cyberarm.engine.V2.CyberarmState;

public class Break extends CyberarmState {

    public DcMotor driveBackLeft;
    public DcMotor driveFrontLeft;
    public DcMotor driveBackRight;
    public DcMotor driveFrontRight;

    @Override
    public void init() {
        driveFrontLeft = engine.hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = engine.hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = engine.hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = engine.hardwareMap.dcMotor.get("driveBackRight");

        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void exec() {
        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);
    }
}
