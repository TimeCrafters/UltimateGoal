package org.timecrafters.javaClass.cayden;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.cyberarm.engine.V2.CyberarmEngine;

@TeleOp(name = "Caydens minibot", group = "Cayden")

    public class miniboi extends CyberarmEngine {
        public DcMotor driveleft;
        public DcMotor driveright;

        public void hardwareInt() {

        }

        @Override
        public void setup() {
            driveleft = hardwareMap.dcMotor.get("frontLeft");
            driveright = hardwareMap.dcMotor.get("frontRight");
            addState(new Minibot_State(this));
        }
    }
