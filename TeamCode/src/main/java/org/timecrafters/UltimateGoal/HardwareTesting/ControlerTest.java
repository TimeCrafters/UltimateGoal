package org.timecrafters.UltimateGoal.HardwareTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Robot;


public class ControlerTest extends CyberarmState {

    private float leftJoystickDegrees;
    private double leftJoystickMagnitude;
    private float rightJoystickDegrees;
    private double rightJoystickMagnitude;
    private double leftJoystickX;
    private double leftJoystickY;
    private double rightJoystickX;
    private double rightJoystickY;




    @Override
    public void exec() {

        leftJoystickX = engine.gamepad1.left_stick_x;
        leftJoystickY = engine.gamepad1.left_stick_y;

        leftJoystickDegrees = (float) Math.toDegrees(Math.atan2(leftJoystickX, -leftJoystickY));
        leftJoystickMagnitude = Math.hypot(leftJoystickX, leftJoystickY);

        rightJoystickX = engine.gamepad1.left_stick_x;
        rightJoystickY = engine.gamepad1.left_stick_y;

        rightJoystickDegrees = (float) Math.toDegrees(Math.atan2(rightJoystickX, -rightJoystickY));
        rightJoystickMagnitude = Math.hypot(rightJoystickX, rightJoystickY);



    }

    @Override
    public void telemetry() {

        engine.telemetry.addLine("Left Joystick");
        engine.telemetry.addData("Pos", "("+leftJoystickX+","+leftJoystickY+")");
        engine.telemetry.addData("Angle", leftJoystickDegrees);
        engine.telemetry.addData("Mag", leftJoystickMagnitude);

        engine.telemetry.addLine("Right Joystick");
        engine.telemetry.addData("Pos", "("+rightJoystickX+","+rightJoystickY+")");
        engine.telemetry.addData("Angle", leftJoystickDegrees);
        engine.telemetry.addData("Mag", leftJoystickMagnitude);

        engine.telemetry.addLine("Buttons");
        engine.telemetry.addData("a", engine.gamepad1.a);
        engine.telemetry.addData("b", engine.gamepad1.b);
        engine.telemetry.addData("x", engine.gamepad1.x);
        engine.telemetry.addData("y", engine.gamepad1.y);

        engine.telemetry.addLine("Top");
        engine.telemetry.addData("Left Bump", engine.gamepad1.left_bumper);
        engine.telemetry.addData("Right Bump", engine.gamepad1.right_bumper);
        engine.telemetry.addData("Left Trigger", engine.gamepad1.left_trigger);
        engine.telemetry.addData("Right Trigger", engine.gamepad1.right_trigger);

    }
}
