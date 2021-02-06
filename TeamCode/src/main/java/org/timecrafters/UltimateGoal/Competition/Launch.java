package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class Launch extends CyberarmState {

    private Robot robot;
    boolean hasCycled;
    boolean detectedPass;

    public Launch(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.ringBeltOn();
    }

    @Override
    public void exec() {
        //detect when limit switch is initially triggered
        boolean detectingPass = robot.limitSwitch.isPressed();
        if (detectingPass && !detectedPass) {
            //finish once the ring belt has cycled all the way through and then returned to
            //the first receiving position.
            if (hasCycled) {
                robot.ringBeltMotor.setPower(0);
                robot.ringBeltStage = 0;
                setHasFinished(true);
            } else {
                hasCycled = true;
            }
        }
        detectedPass = detectingPass;

//        if (robot.getBeltPos() > robot.loopPos(Robot.RING_BELT_GAP * 3) && hasCycled);
    }
}
