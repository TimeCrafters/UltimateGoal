package org.timecrafters.javaClass.cayden;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.UltimateGoal.Competition.Robot;

public class Cayden_Autonomous extends CyberarmState {
    private Robot robot;
    private double variable=.5;




    public Cayden_Autonomous(Robot robot) {
        this.robot = robot;
    }
    @Override
    public void exec() {

        robot.driveFrontRight.setPower(variable);
        robot.driveBackRight.setPower(variable);
        robot.driveFrontLeft.setPower(variable);
        robot.driveBackLeft.setPower(variable);

        if (robot.encoderLeft.getCurrentPosition()>=300){
            variable=0;
        }


    }
}
