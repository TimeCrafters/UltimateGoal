package org.cyberarm.engine.V2;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.concurrent.CopyOnWriteArrayList;

/**
 * CyberarmEngine Version 2.0 | October 26th 2018
 * AN Experimental reimplementation of GoldfishPi's original Engine system.
 * Designed to be easily maintainable, extensible, and easy to understand.
 */
public abstract class CyberarmEngine extends OpMode {

  public static CyberarmEngine instance;
  //Array To Hold States
  private CopyOnWriteArrayList<CyberarmState> cyberarmStates = new CopyOnWriteArrayList<>();
  private int activeStateIndex = 0;
  private boolean isRunning;

  private static String TAG = "PROGRAM.ENGINE";
  public boolean showStateChildrenListInTelemetry = false;

  /**
   * Called when INIT button on Driver Station is pushed
   * ENSURE to call super.init() if you override this method
   */
  public void init() {
    CyberarmEngine.instance = this;
    isRunning = false;

    setup();

    isRunning = true;

    for (CyberarmState state: cyberarmStates) {
      initState(state);
    }
  }

  /**
   * Setup states for engine to use
   * For example:
   * <pre>
   * @<code>
   *   public void setup() {
   *     addState(new TestState());
   *     addState(new AnotherState(100, 500));
   *   }
   * </code>
   * </pre>
   */
  public abstract void setup();

  /**
   * Called when START button on Driver Station is pushed
   * ENSURE to call super.start() if you override this method
   */
  public void start() {
    if (cyberarmStates.size() > 0) {
      runState(cyberarmStates.get(0));
    }
  }

  /**
   * Engine main loop
   * ENSURE to call super.loop() if you override this method
   */
  public void loop() {
    CyberarmState state;

    // Try to set state to the current state, if it fails assume that there are no states to run
    try {
       state = cyberarmStates.get(activeStateIndex);
    } catch(IndexOutOfBoundsException e) {
      // The engine is now out of states.
      stop();

      telemetry.addLine("" + this.getClass().getSimpleName() + " is out of states to run!");
      telemetry.addLine();
      return;
    }

      // Add telemetry to show currently running state
    telemetry.addLine("Running state: " +state.getClass().getSimpleName() + ". State: " + activeStateIndex + " of " + (cyberarmStates.size()-1));
    if (showStateChildrenListInTelemetry && state.hasChildren()) {
      for(CyberarmState child: state.children) {
        telemetry.addLine("    Child: " + child.getClass().getSimpleName() + " [" + child.children.size() + "] grandchildren");
      }
    }
    telemetry.addLine();

    if (state.getHasFinished() && state.childrenHaveFinished()) {
      activeStateIndex++;

      try {
        state = cyberarmStates.get(activeStateIndex);
        runState(state);
      } catch(IndexOutOfBoundsException e) { /* loop will handle this in a few milliseconds */ }

    } else {
      stateTelemetry(state);
    }
  }

  /**
   * Stops every known state
   */
  @Override
  public void stop() {
    for (CyberarmState state: cyberarmStates) {
      stopState(state);
    }


  }

  /**
   * Recursively calls telemetry() on states
   * @param state State to get telemetry
   */
  private void stateTelemetry(CyberarmState state) {
    if (!state.getHasFinished()) {
      state.telemetry();
    }

    for(CyberarmState childState : state.children) {
      if (!childState.getHasFinished()) {
        stateTelemetry(childState);
      }
    }
  }

  /**
   * Called when INIT button on Driver Station is pressed
   * Recursively initiates states
   * @param state State to initiate
   */
  private void initState(CyberarmState state) {
    state.init();

    for(CyberarmState childState : state.children) {
      initState(childState);
    }
  }

  /**
   * Called when programs ends or STOP button on Driver Station is pressed
   * Recursively stop states
   * @param state State to stop
   */
  public void stopState(CyberarmState state) {
    state.setHasFinished(true);
    state.stop();

    for(CyberarmState childState : state.children) {
      stopState(childState);
    }
  }

  /**
   * Recursively start up states
   * @param state State to run
   */
  protected void runState(CyberarmState state) {
    final CyberarmState finalState = state;
//    if (state.isRunning()) { return; } // Assume that we have already started running this state

    new Thread(new Runnable() {
      @Override
      public void run() {
        finalState.prestart();
        finalState.start();
        finalState.startTime = System.currentTimeMillis();
        finalState.run();
      }
    }).start();

    for (CyberarmState kid : state.children) {
      runState(kid);
    }
  }

  /**
   * Add state to queue, will call init() on state if engine is running
   * @param state State to add to queue
   */
  public CyberarmState addState(CyberarmState state) {
    Log.i(TAG, "Adding cyberarmState "+ state.getClass());
    cyberarmStates.add(state);

    if (isRunning()) { initState(state); }

    return state;
  }

  /**
   * Inserts state after the query state plus an offset to ensure logical insertion
   * @param query State to add state after
   * @param state State to be inserted
   * @return
   */
  public CyberarmState insertState(CyberarmState query, CyberarmState state) {
    int index = cyberarmStates.indexOf(query) + query.insertOffset;
    Log.i(TAG, "Adding cyberarmState "+ state.getClass());

    cyberarmStates.add(index, state);
    query.insertOffset++;

    if (isRunning()) { initState(state); }

    return state;
  }

  /**
   * This will return false while Engine.setup() is executing, and be true after.
   * @return Whether the engine main loop is running
   */
  public boolean isRunning() {
    return isRunning;
  }

  /**
   *
   * @return The index used to lookup the current state from cyberarmStates
   */
  public int getActiveStateIndex() {
    return activeStateIndex;
  }
}
