package org.timecrafters.javaClass.spencer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spencer_Dmitry {
    private HardwareMap hardwareMap;

    public Spencer_Dmitry(HardwareMap hardWareMap) {
        hardwareMap = hardWareMap;
    }

    public DcMotor driveleft;
    public DcMotor driveright;

    public void hardwareInt() {
        driveleft = hardwareMap.dcMotor.get("frontLeft");
        driveright = hardwareMap.dcMotor.get("frontRight");
    }




}
