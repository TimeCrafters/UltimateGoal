/*package org.timecrafters.javaClass.spencer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.cyberarm.engine.V2.CyberarmState;

@TeleOp (name = "Spencer: Tank Drive")

public class spencerTankDrive extends CyberarmState {

    private Spencer_Dmitry spencer_dmitry;
    private double leftstick;
    private double rightstick;

    @Override
    public void init() {
        spencer_dmitry = new Spencer_Dmitry(engine.hardwareMap);
        spencer_dmitry.hardwareInt();
    }

    @Override
    public void exec() {
        spencer_dmitry.driveleft.setPower(engine.gamepad1.left_stick_y);
        spencer_dmitry.driveright.setPower(engine.gamepad1.right_stick_y);
    }
}
*/