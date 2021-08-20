package org.timecrafters.javaClass.cayden;

import org.cyberarm.engine.V2.CyberarmEngine;

public class Cayden_Dimitri_engine extends CyberarmEngine {

  private Cayden_Dimitri cayden_dimitri;

    @Override
    public void init() {
        cayden_dimitri = new Cayden_Dimitri(hardwareMap);
        super.init();
    }

    @Override
    public void setup() {
addState(new Cayden_Dimitri_Move(cayden_dimitri));
    }
}
