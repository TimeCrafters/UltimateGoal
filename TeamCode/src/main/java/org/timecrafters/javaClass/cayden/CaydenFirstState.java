package org.timecrafters.javaClass.cayden;

import org.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.javaClass.samples.SampleRobot;

public class CaydenFirstState extends CyberarmState {

    //here, you'll find some of your variables. you can add more as you need them.
    private SampleRobot robot;

    //This is the constructor. It lets other code bits run use the code you put here
    public CaydenFirstState(SampleRobot robot) {
        this.robot = robot;
    }

    //This is a method. methods are bits of code that can be run elsewhere. 
    //This one is set up to repeat every few milliseconds
    @Override
    public void exec() {

        if (engine.gamepad1.y){
            robot.collectionMotor.setPower(1);
        }
        else{
            robot.collectionMotor.setPower(0);
        }

        if(engine.gamepad1.b){
            robot.ringBeltMotor.setPower(.1);
        }
        else{
            robot.ringBeltMotor.setPower(0);
        }


        robot.driveFrontRight.setPower(-engine.gamepad1.right_stick_y);
        robot.driveBackRight.setPower(-engine.gamepad1.right_stick_y);
        robot.driveFrontLeft.setPower(-engine.gamepad1.left_stick_y);
        robot.driveBackLeft.setPower(-engine.gamepad1.right_stick_y);

        if (engine.gamepad1.left_bumper) {
            robot.driveFrontLeft.setPower(-1);
            robot.driveFrontRight.setPower(-1);
            robot.driveBackLeft.setPower(-1);
            robot.driveBackRight.setPower(-1);
        } else if (engine.gamepad1.right_bumper) {
            robot.driveFrontLeft.setPower(1);
            robot.driveFrontRight.setPower(1);
            robot.driveBackLeft.setPower(1);
            robot.driveBackRight.setPower(1);
        }

        robot.ringBeltMotor.setPower(engine.gamepad1.right_trigger);

        if (engine.gamepad1.x){
            robot.launchMotor.setPower(1);
        }else{
            robot.launchMotor.setPower(0);
        }
    }
}
