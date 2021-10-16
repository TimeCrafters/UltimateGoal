package org.timecrafters.javaClass.cayden;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.cyberarm.engine.V2.CyberarmEngine;
import org.timecrafters.UltimateGoal.Competition.Robot;
import org.timecrafters.javaClass.cayden.CaydenFirstState;
import org.timecrafters.javaClass.cayden.Cayden_Autonomous;

@Autonomous (name = "Cayden: First Program", group = "caden")
public class CaydenFirstEngine extends CyberarmEngine {
    
   Robot robot;



    @Override
    public void setup() {
        addState(new CaydenFirstState());
    }
}
