package org.timecrafters.UltimateGoal.HardwareTesting;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.cyberarm.engine.V2.CyberarmState;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DistanceSensorTest extends CyberarmState {


    private Rev2mDistanceSensor distanceSensor;
//    private RevColorSensorV3 colorSensor;
    private ColorRangeSensor colorSensor;
    private double distance = 0;
    private double color = 0;

    @Override
    public void init() {
        distanceSensor = engine.hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        colorSensor = engine.hardwareMap.get(ColorRangeSensor.class, "color");
//        colorSensor = engine.hardwareMap.get(RevColorSensorV3.class, "color");
    }

    @Override
    public void exec() {
        distance = distanceSensor.getDistance(DistanceUnit.MM);
        color = colorSensor.getDistance(DistanceUnit.MM);
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("distance", distance);
        engine.telemetry.addData("color", color);
    }
}
