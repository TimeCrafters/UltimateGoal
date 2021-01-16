package org.timecrafters.UltimateGoal.HardwareTesting;

import org.cyberarm.engine.V2.CyberarmState;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.timecrafters.UltimateGoal.Competition.Robot;


public class ControlHubTest extends CyberarmState {

    private Robot robot;

    private Acceleration acceleration;
    private double Vx = 0;
    private double Vy = 0;
    private double Vz = 0;
    private double aVx = 0;
    private double aVy = 0;
    private double aVz = 0;


    public ControlHubTest(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {

        Velocity startVelocity = new Velocity();
        startVelocity.xVeloc = 0;
        startVelocity.yVeloc = 0;
        startVelocity.zVeloc = 0;

        Position startPosition = new Position();
        startPosition.x = 0;
        startPosition.y = 0;
        startPosition.z = 0;

        robot.imu.startAccelerationIntegration(startPosition,startVelocity, 200);
    }

    @Override
    public void exec() {
        acceleration = robot.imu.getAcceleration();

        Vx = robot.imu.getVelocity().xVeloc;
        Vy = robot.imu.getVelocity().yVeloc;
        Vz = robot.imu.getVelocity().zVeloc;

        aVx = robot.imu.getAngularVelocity().xRotationRate;
        aVy = robot.imu.getAngularVelocity().yRotationRate;
        aVz = robot.imu.getAngularVelocity().zRotationRate;


    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("Acceleration");
        engine.telemetry.addData("x", acceleration.xAccel);
        engine.telemetry.addData("y", acceleration.yAccel);
        engine.telemetry.addData("z", acceleration.zAccel);

        engine.telemetry.addLine("Velocity");
        engine.telemetry.addData("X", Vx);
        engine.telemetry.addData("Y", Vy);
        engine.telemetry.addData("Z", Vz);

        engine.telemetry.addLine("Angular Velocity");
        engine.telemetry.addData("X", aVx);
        engine.telemetry.addData("Y", aVy);
        engine.telemetry.addData("Z", aVz);

    }
}
