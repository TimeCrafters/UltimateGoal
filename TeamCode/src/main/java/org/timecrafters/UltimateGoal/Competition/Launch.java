package org.timecrafters.UltimateGoal.Competition;

import org.cyberarm.engine.V2.CyberarmState;

public class Launch extends CyberarmState {

    private Robot robot;
    private String groupName;
    private String actionName;
    boolean hasCycled;
    boolean detectedPass;
    private boolean reduceConditionPrev;
    private double reducePos;
    private long stuckStartTime;

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
                robot.ringBeltMotor.setPower(0.5);
            } else {
                setHasFinished(true);
            }
        } catch (NullPointerException e){
            robot.ringBeltMotor.setPower(0.5);
        }
    }

    @Override
    public void exec() {
        //detect when limit switch is initially triggered
        boolean detectingPass = robot.limitSwitch.isPressed();
        int beltPos = robot.ringBeltMotor.getCurrentPosition();

        if (robot.beltIsStuck() && childrenHaveFinished()) {
            long currentTime = System.currentTimeMillis();
            if (stuckStartTime == 0) {
                stuckStartTime = currentTime;
            } else if (currentTime - stuckStartTime >= robot.beltMaxStopTime) {
                addParallelState(new UnstickRingBelt(robot));
            }
        } else {
            stuckStartTime = 0;
        }

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
                reducePos = robot.loopPos((int) (beltPos + (1.5 * Robot.RING_BELT_GAP)));
            }
        }
        detectedPass = detectingPass;

        boolean reduceCondition = (hasCycled && beltPos > reducePos);
        if (reduceCondition && !reduceConditionPrev){
            robot.ringBeltOn();
            //the ring belt stage lets other states know that the robot has finished launching all three rings
            //and is now returning to loading position.

            robot.ringBeltStage += 1;
        }
        reduceConditionPrev = reduceCondition;
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("hasCycled", hasCycled);
    }
}
