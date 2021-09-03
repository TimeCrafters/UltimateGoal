package org.timecrafters.javaClass.spencer;

import org.cyberarm.engine.V2.CyberarmState;

public class spencerTurnState extends CyberarmState {
   private Spencer_Dmitry spencer_dmitry;
   public spencerTurnState (Spencer_Dmitry spencer_dmitry) {
       this.spencer_dmitry = spencer_dmitry;
   }

    @Override
    public void start() {

   }


    @Override
    public void exec() {
        spencer_dmitry.driveleft.setPower(-.5);
        spencer_dmitry.driveright.setPower(.5);
        sleep(1000);
        stop();
    }
}
