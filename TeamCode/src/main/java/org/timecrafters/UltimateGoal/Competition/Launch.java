package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class Launch extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    boolean hasCycled;
    boolean detectedPass;

    public Launch(Robot robot) {
        this.robot = robot;
    }

    public Launch(Robot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;
    }

    @Override
    public void start() {
        try {
            if (robot.stateConfiguration.action(groupName, actionName).enabled) {
                robot.ringBeltOn();
            } else {
                setHasFinished(true);
            }
        } catch (NullPointerException e){
            robot.ringBeltOn();
        }
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

                if (!robot.initLauncher) {
                    robot.launchMotor.setPower(0);
                }

                setHasFinished(true);
            } else {
                hasCycled = true;
            }
        }
        detectedPass = detectingPass;

    }
}
