package org.timecrafters.UltimateGoal.Calibration;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

import java.util.ArrayList;

public class MechanumBiasCalibrator extends CyberarmState {

    private Robot robot;
    private ArrayList<double[]> Powers = new ArrayList<>();
    private double BiasFR;
    private double BiasFL;
    private double BiasBR;
    private double BiasBL;
    private boolean hasCalculated;


    public MechanumBiasCalibrator(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.updateLocation();

        if (engine.gamepad1.x) {
            double[] mecanumPowers = robot.getMecanumPowers(-45, 0.5, 0);

            
            robot.driveFrontLeft.setPower(mecanumPowers[0]);
            robot.driveFrontRight.setPower(mecanumPowers[1]);
            robot.driveBackLeft.setPower(mecanumPowers[2]);
            robot.driveBackRight.setPower(mecanumPowers[3]);

            Powers.add(mecanumPowers);
        } else {
            robot.setDrivePower(0,0,0,0 );
        }

        if (engine.gamepad1.y && !hasCalculated) {
            hasCalculated = true;

            double sumFR = 0;
            double sumFL = 0;
            double sumBR = 0;
            double sumBL = 0;

            for (double[] powers : Powers) {
                sumFL+= Math.abs(powers[0]);
                sumFR+= Math.abs(powers[1]);
                sumBL+= Math.abs(powers[2]);
                sumBR+= Math.abs(powers[3]);
            }

            int length = Powers.size();
            BiasFR = sumFR / length;
            BiasFL = sumFL / length;
            BiasBR = sumBR / length;
            BiasBL = sumBL / length;

            double max = Math.max(Math.max(BiasFL,BiasFR),Math.max(BiasBL,BiasBR));

            BiasFL = BiasFL /max;
            BiasFR = BiasFR /max;
            BiasBL = BiasBL /max;
            BiasBR = BiasBR /max;

        } else if (!engine.gamepad1.y) {
            hasCalculated = false;
        }

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("FrontLeft", BiasFL);
        engine.telemetry.addData("FrontRight", BiasFR);
        engine.telemetry.addData("BackLeft", BiasBL);
        engine.telemetry.addData("BackRight", BiasBR);
    }
}
