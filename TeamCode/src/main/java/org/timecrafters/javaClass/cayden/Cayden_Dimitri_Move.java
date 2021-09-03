package org.timecrafters.javaClass.cayden;

import org.cyberarm.engine.V2.CyberarmState;

public class Cayden_Dimitri_Move extends CyberarmState {

    private Cayden_Dimitri robot;

    public Cayden_Dimitri_Move(Cayden_Dimitri robot) {
        this.robot = robot;
    }
    @Override
    public void exec() {
    robot.driveRight.setPower(.75);
    robot.driveLeft.setPower(.75);

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("arm position",robot.arm.getCurrentPosition());
    }
}
