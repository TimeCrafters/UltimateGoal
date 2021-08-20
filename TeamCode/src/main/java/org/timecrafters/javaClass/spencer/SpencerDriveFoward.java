package org.timecrafters.javaClass.spencer;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class SpencerDriveFoward extends CyberarmState {


    private Spencer_Dmitry spencer_dmitry;

    public SpencerDriveFoward(Spencer_Dmitry spencer_dmitry) {
        this.spencer_dmitry = spencer_dmitry;
    }



    @Override
    public void exec() {

    }

    @Override
    public void start() {
        super.start();
        spencer_dmitry.driveright.setPower(0.5);
        spencer_dmitry.driveleft.setPower(0.5);
    }
}
