package org.timecrafters.javaClass.spencer;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class SpencerDriveFoward extends CyberarmState {


    private Spencer_Dmitry spencer_dmitry;
    public int ourposition;

    public SpencerDriveFoward(Spencer_Dmitry spencer_dmitry) {
        this.spencer_dmitry = spencer_dmitry;
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {
        spencer_dmitry.driveleft.setPower(.5);
    }

    @Override
    public void exec() {
        if (spencer_dmitry.driveleft.getCurrentPosition() >= ourposition){
            spencer_dmitry.driveleft.setPower(0);
    }
}
}
