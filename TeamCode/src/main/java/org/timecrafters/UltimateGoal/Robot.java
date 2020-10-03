package org.timecrafters.UltimateGoal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.cyberarm.NeXT.StateConfiguration;

public class Robot {

    private HardwareMap hardwareMap;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public StateConfiguration stateConfiguration = new StateConfiguration();
    public BNO055IMU imu;

    //drive system

    public DcMotor encoderFront;
    public DcMotor encoderLeft;
    public DcMotor encoderBack;
    public DcMotor encoderRight;

    double BIAS_LEFT = -1.0;
    double BIAS_RIGHT = -0.87;

    double Circumference_Encoder = Math.PI * 3.8;
    int Counts_Per_Revolution = 8192;

    //Robot Localizatoin
    private double locationX;
    private double locationY;
    private float rotation;

    private int encoderFrontPrevious = 0;
    private int encoderLeftPrevious = 0;
    private int encoderBackPrevious = 0;
    private int encoderRightPrevious = 0;
    private float rotationPrevious = 0;

    public void initHardware() {
        imu  = hardwareMap.get(BNO055IMU.class, "imu");
//        encoderFront = hardwareMap.dcMotor.get("encoderFront");
        encoderLeft = hardwareMap.dcMotor.get("encoderLeft");
//        encoderBack = hardwareMap.dcMotor.get("encoderBack");
        encoderRight = hardwareMap.dcMotor.get("encoderRight");

        encoderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        encoderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
    }

    public void setDrivePower(double powerLeft, double powerRight){
        encoderLeft.setPower(powerLeft * BIAS_LEFT);
        encoderRight.setPower(powerRight * BIAS_RIGHT);

    }

    public void updateLocation(){
        rotation = imu.getAngularOrientation().firstAngle;
        float rotationChange = rotation - rotationPrevious;
        int encoderLeftCurrent = encoderLeft.getCurrentPosition();
        int encoderRightCurrent = encoderRight.getCurrentPosition();
        double encoderLeftChange = encoderLeftCurrent - encoderLeftPrevious;
        double encoderRightChange = encoderRightCurrent - encoderRightPrevious;

        encoderLeftPrevious = encoderLeftCurrent;
        encoderRightPrevious = encoderRightCurrent;
        rotationPrevious = rotation;

        double average = (encoderLeftChange+encoderRightChange)/2;

        double xChange = average * (Math.sin(Math.toRadians(rotation)));
        double yChange = average * (Math.cos(Math.toRadians(rotation)));

        locationX += xChange;
        locationY += yChange;

    }

    public double getRotation() {
        return rotation;
    }

    public double getLocationX() {
        return locationX;
    }

    public double getLocationY() {
        return locationY;
    }

    public double ticksToInches(double ticks) {
        return ticks * (Circumference_Encoder / Counts_Per_Revolution);
    }

    public double inchesToTicks(double inches) {
        return inches * (Counts_Per_Revolution / Circumference_Encoder);
    }
}
