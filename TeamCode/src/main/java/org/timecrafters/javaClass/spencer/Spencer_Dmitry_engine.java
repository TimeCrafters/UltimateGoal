package org.timecrafters.javaClass.spencer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.cyberarm.engine.V2.CyberarmEngine;

@Autonomous (name = "Spencer: Drive Foward")

public class Spencer_Dmitry_engine extends CyberarmEngine {

    private Spencer_Dmitry spencer_dmitry;


    @Override
    public void init() {
        spencer_dmitry = new Spencer_Dmitry(hardwareMap);
        spencer_dmitry.hardwareInt();
        super.init();
    }

    @Override
    public void setup() {
    addState(new SpencerDriveFoward(spencer_dmitry));
    }
}
