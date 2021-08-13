package org.timecrafters.UltimateGoal.Competition;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.cyberarm.engine.V2.CyberarmState;

public class ResetRingBelt extends CyberarmState {

    private Robot robot;
    boolean detectedPass;
    int reducePos;
    boolean reduceConditionPrev;

    public ResetRingBelt(Robot robot) {
        this.robot = robot;
    }


    @Override
    public void start() {
        robot.ringBeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ringBeltMotor.setPower(Robot.RING_BELT_NORMAL_POWER);
        reducePos = robot.ringBeltMotor.getCurrentPosition() + robot.reduceLaunchPos;
    }

    @Override
    public void exec() {
        //detect when limit switch is initially triggered
        boolean detectingPass = robot.magnetSensor.isPressed();
        int beltPos = robot.ringBeltMotor.getCurrentPosition();

        if (detectingPass && !detectedPass) {
            //finish once the ring belt has cycled all the way through and then returned to
            //the first receiving position.


            robot.ringBeltStage = 0;
            robot.ringBeltMotor.setTargetPosition(beltPos);
            robot.ringBeltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.launchMotor.setPower(0);
            setHasFinished(true);
        }
        detectedPass = detectingPass;

        boolean reduceCondition = (beltPos > reducePos);
        if (reduceCondition && !reduceConditionPrev){
            robot.ringBeltMotor.setPower(Robot.RING_BELT_SLOW_POWER);

            //the ring belt stage lets other states know that the robot has finished launching all three rings
            //and is now returning to loading position.

        }
        reduceConditionPrev = reduceCondition;
    }

}
