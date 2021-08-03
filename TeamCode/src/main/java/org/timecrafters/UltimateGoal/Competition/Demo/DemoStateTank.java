package org.timecrafters.UltimateGoal.Competition.Demo;

/*
The Player1 state has all the controls for player one. The Player One and Player Two controls are
kept in separate states so that the childrenHaveFinished() method can be used to easily stop things
from running at the same time that shouldn't be.
*/

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Autonomous.FindWobbleGoal;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.UltimateGoal.Competition.TeleOp.powerShotsControl;

public class DemoStateTank extends CyberarmState {
    private Robot robot;

    private double drivePower = 1;
    private boolean lbPrev;


    public DemoStateTank(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.updateLocation();

        Gamepad gamepad = engine.gamepad1;
        if (engine.gamepad2.right_bumper) {
            gamepad = engine.gamepad2;
        }

        //toggle for drive speed
        boolean lb = engine.gamepad2.left_stick_button;
        if (lb && !lbPrev) {
            if (drivePower == 1) {
                drivePower = 0.5;
            } else {
                drivePower = 1;
            }
        }
        lbPrev = lb;

        double left = -drivePower * gamepad.left_stick_y;
        double right = -drivePower * gamepad.right_stick_y;
        robot.setDrivePower(left,right,left,right);

    }

}
