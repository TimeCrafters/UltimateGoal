package org.timecrafters.UltimateGoal.Autonomous;

import org.cyberarm.engine.V2.CyberarmState;

import java.util.ArrayList;

public class ThreadStates extends CyberarmState {

    private ArrayList<CyberarmState> states = new ArrayList<>();

    public ThreadStates(ArrayList<CyberarmState> states) {
        this.states = states;
    }

    @Override
    public void start() {
        for (CyberarmState state : states) {
            addParallelState(state);
        }
    }

    @Override
    public void exec() {
        int finishedStates = 0;
        for (CyberarmState state : states) {
            if (state.getHasFinished()) {
                finishedStates += 1;
            }
        }
        setHasFinished(finishedStates == states.size());

    }
}
