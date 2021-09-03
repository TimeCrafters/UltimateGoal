package org.timecrafters.javaClass.cayden;

import org.cyberarm.engine.V2.CyberarmState;

public class Minibot_State extends CyberarmState {

    private Minibot_State robot;

    public Minibot_State(Minibot_State robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        if (engine.gamepad1.right_trigger > -0.5) {
            robot.driveleft.setPower(1);
        } else robot.driveleft.setPower(0);
    }
}