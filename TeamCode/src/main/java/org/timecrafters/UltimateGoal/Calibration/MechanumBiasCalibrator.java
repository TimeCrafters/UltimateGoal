package org.timecrafters.UltimateGoal.Calibration;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class MechanumBiasCalibrator extends CyberarmState {

    private Robot robot;
    private ArrayList<double[]> Powers;
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
            double[] mecanumPowers = robot.getMecanumPowers(0, 1, 0);

            //TODO: add motors when they exist.

            Powers.add(mecanumPowers);
        }

        if (engine.gamepad1.y && !hasCalculated) {
            hasCalculated = true;

            double sumFR = 0;
            double sumFL = 0;
            double sumBR = 0;
            double sumBL = 0;

            for (double[] powers : Powers) {
                sumFR+= powers[0];
                sumFL+= powers[1];
                sumBR+= powers[2];
                sumBL+= powers[3];
            }

            int length = Powers.size();
            BiasFR = sumFR / length;
            BiasFL = sumFL / length;
            BiasBR = sumBR / length;
            BiasBL = sumBL / length;

        } else if (!engine.gamepad1.y) {
            hasCalculated = false;
        }

    }
}
