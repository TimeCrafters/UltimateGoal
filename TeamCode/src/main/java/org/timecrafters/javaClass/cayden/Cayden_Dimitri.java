package org.timecrafters.javaClass.cayden;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Cayden_Dimitri {
    private HardwareMap hardwareMap;

    public Cayden_Dimitri(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public DcMotor driveLeft;
    public DcMotor arm;
    public DcMotor driveRight;

    public void hardware_init(){

arm = hardwareMap.dcMotor.get("arm motor");
driveLeft = hardwareMap.dcMotor.get("frontLeft");
driveRight = hardwareMap.dcMotor.get("frontRight");

    }
}
