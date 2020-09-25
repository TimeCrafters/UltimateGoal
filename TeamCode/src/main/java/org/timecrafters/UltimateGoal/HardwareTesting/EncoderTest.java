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

        robot.setDrivePower(-0.2 * engine.gamepad1.left_stick_y, -0.2 * engine.gamepad1.right_stick_y);

        ticksLeft=robot.encoderLeft.getCurrentPosition();
        ticksRight=robot.encoderRight.getCurrentPosition();


//        if (runTime() < 3000) {
//
//
//
//
//        } else {
//            robot.encoderLeft.setPower(0.0);
//            robot.encoderRight.setPower(0.0);
//
//            double ticksExtreme;
//            if (Math.abs(ticksLeft) < Math.abs(ticksRight)) {
//                ticksExtreme = ticksLeft;
//            } else {
//                ticksExtreme = ticksRight;
//            }
//            biasLeft = ticksExtreme/ticksLeft;
//            biasRight = ticksExtreme/ticksRight;
//        }

    }

    @Override
    public void telemetry() {

        engine.telemetry.addData("controler", engine.gamepad1.left_stick_y);
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
