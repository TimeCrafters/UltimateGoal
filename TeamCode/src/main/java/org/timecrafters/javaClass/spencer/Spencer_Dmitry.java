package org.timecrafters.javaClass.spencer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spencer_Dmitry {
    private HardwareMap hardwareMap;

    public Spencer_Dmitry(HardwareMap hardWareMap) {
        hardwareMap = hardWareMap;
    }

    public DcMotor driveleft;
    public DcMotor driveright;
    public DcMotor armmotor;

    public void hardwareInt() {
        driveleft = hardwareMap.dcMotor.get("frontLeft");
        driveright = hardwareMap.dcMotor.get("frontRight");
        armmotor = hardwareMap.dcMotor.get("arm motor");

        driveleft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveright.setDirection(DcMotorSimple.Direction.FORWARD);
    }


}
