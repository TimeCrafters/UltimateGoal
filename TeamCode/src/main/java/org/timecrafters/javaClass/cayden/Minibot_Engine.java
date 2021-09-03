package org.timecrafters.javaClass.cayden;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Cayden: First Program", group = "caden")

    public class miniboi extends CyberarmEngine {
        private HardwareMap hardwareMap;
        public miniboi(HardwareMap hardWareMap) {
            hardwareMap = hardWareMap;
        }

        public DcMotor driveleft;
        public DcMotor driveright;

        public void hardwareInt() {
            driveleft = hardwareMap.dcMotor.get("frontLeft");
            driveright = hardwareMap.dcMotor.get("frontRight");
            addState(new Minibot_State(robot,1,12));
        }
    
        @Override
        public void setup() {

        }
    }
