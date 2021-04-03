package org.timecrafters.UltimateGoal.HardwareTesting;

import com.qualcomm.robotcore.hardware.Servo;

import org.cyberarm.engine.V2.CyberarmState;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.timecrafters.UltimateGoal.Competition.Robot;


public class ServoPosTest extends CyberarmState {

    private Servo servo;
    private double servoPos;

    public ServoPosTest() {
    }

    @Override
    public void init() {
        servo = engine.hardwareMap.servo.get("look");
    }

    @Override
    public void exec() {
        servoPos = engine.gamepad1.right_stick_y;
        servo.setPosition(servoPos);
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Position", servoPos);
    }
}
