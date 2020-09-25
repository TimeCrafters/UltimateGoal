package org.timecrafters.UltimateGoal.HardwareTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class EncoderTest extends CyberarmState {

    private Robot robot;
    private int ticksLeft;
    private int ticksRight;
    private double biasLeft = 0;
    private double biasRight = 0;

    public EncoderTest(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.updateLocation();

        if (runTime() < 3000) {
            robot.setDrivePower(0.5, 0.5);

            ticksLeft=robot.encoderLeft.getCurrentPosition();
            ticksRight=robot.encoderRight.getCurrentPosition();



        } else {
            robot.encoderLeft.setPower(0.0);
            robot.encoderRight.setPower(0.0);

            double ticksExtreme;
            if (Math.abs(ticksLeft) < Math.abs(ticksRight)) {
                ticksExtreme = ticksLeft;
            } else {
                ticksExtreme = ticksRight;
            }
            biasLeft = ticksExtreme/ticksLeft;
            biasRight = ticksExtreme/ticksRight;
        }

    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("Biases");
        engine.telemetry.addData("Left", biasLeft);
        engine.telemetry.addData("Right", biasRight);
        engine.telemetry.addLine();
        engine.telemetry.addLine("Latency Values");
        engine.telemetry.addData("Y", robot.getLocationY());
        engine.telemetry.addData("X", robot.getLocationX());
        engine.telemetry.addLine();
        engine.telemetry.addData("Rotation", robot.getRotation());
        engine.telemetry.addLine();
        engine.telemetry.addLine("Actual Values");
        engine.telemetry.addData("Left", ticksLeft);
        engine.telemetry.addData("Right", ticksRight);
//        engine.telemetry.addLine("");
//        engine.telemetry.addData("Front", robot.encoderFront);
//        engine.telemetry.addData("Back", robot.encoderBack);
    }
}
