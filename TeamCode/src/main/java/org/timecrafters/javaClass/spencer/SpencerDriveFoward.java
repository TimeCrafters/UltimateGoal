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
    spencer_dmitry.armmotor.setPower(1);
    sleep(200);
    }

    @Override
    public void start() {
        spencer_dmitry.armmotor.setPower(1);
        sleep(100);
        spencer_dmitry.driveleft.setPower(-1);
        spencer_dmitry.driveright.setPower(-1);
        sleep(4800);
        spencer_dmitry.driveleft.setPower(0);
        spencer_dmitry.driveright.setPower(0);

        spencer_dmitry.driveleft.setPower(-1);
        spencer_dmitry.driveright.setPower(1);
        sleep(850);

        spencer_dmitry.driveleft.setPower(-1);
        spencer_dmitry.driveright.setPower(-1);
        sleep(2500);

        spencer_dmitry.driveleft.setPower(-1);
        spencer_dmitry.driveright.setPower(1);
        sleep(850);

        spencer_dmitry.driveleft.setPower(-1);
        spencer_dmitry.driveright.setPower(-1);
        sleep(3700);

        spencer_dmitry.driveleft.setPower(-1);
        spencer_dmitry.driveright.setPower(1);
        sleep(850);

        spencer_dmitry.driveleft.setPower(-1);
        spencer_dmitry.driveright.setPower(-1);
        sleep(2500);
    }

    @Override
    public void exec() {
        spencer_dmitry.driveleft.setPower(0);
        spencer_dmitry.driveright.setPower(0);
    }
}
