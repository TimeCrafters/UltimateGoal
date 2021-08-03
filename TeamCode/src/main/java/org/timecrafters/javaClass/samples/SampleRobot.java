 package org.timecrafters.javaClass.samples;

/*
The robot object contains all the hardware and functions that are used in both teleOp and
Autonomous. This includes drive functions, localization functions, shared constants, and a few
general calculations and debugging tools.
*/

import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.timecrafters.TimeCraftersConfigurationTool.TimeCraftersConfiguration;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

 public class SampleRobot {

     private HardwareMap hardwareMap;

     public SampleRobot(HardwareMap hardwareMap) {
         this.hardwareMap = hardwareMap;
     }

     //The TimeCraftersConfiguration is part of a debugging and tuning tool that allows us to edit
     //variables saved on the phone, without having to re-download the whole program. This is
     //especially useful for autonomous route tuning
     public TimeCraftersConfiguration stateConfiguration = new TimeCraftersConfiguration();

     //We use the IMU to get reliable rotation and angular velocity information. Experimentation has
     //demonstrated that the accelerometer and related integrations aren't as accurate.
     public BNO055IMU imu;

     //The LEDs are used to provide driver feedback and for looking beautiful
     public RevBlinkinLedDriver ledDriver;

     //drive and dead-wheal hardware
     public DcMotor driveFrontLeft;
     public DcMotor driveBackLeft;
     public DcMotor driveFrontRight;
     public DcMotor driveBackRight;

     public DcMotor encoderLeft;
     public DcMotor encoderRight;
     public DcMotor encoderBack;

     //Unit Conversion Constants
     static final double ENCODER_CIRCUMFERENCE = Math.PI * 2.3622;
     static final int COUNTS_PER_REVOLUTION = 8192;
     //Launcher
     public DcMotor launchMotor;
     public static final double LAUNCH_POWER = 0.715;

     //Ring Intake
     public DcMotor collectionMotor;

     //Ring Belt
     public DcMotor ringBeltMotor;
     public RevTouchSensor limitSwitch;

     //Wobble Goal Arm & Grabber
     public DcMotor  wobbleArmMotor;
     public Servo  wobbleGrabServo;
     public RevColorSensorV3 wobbleColorSensor;
     public RevTouchSensor wobbleTouchSensor;

     //All our hardware initialization in one place, for everything that is the same in TeleOp and
     //Autonomous
     public void initHardware() {

         //drive motors
         driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
         driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
         driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
         driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

         driveFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
         driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
         driveBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
         driveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

         //Localization encoders
         encoderRight = hardwareMap.dcMotor.get("driveFrontRight");
         encoderBack = hardwareMap.dcMotor.get("driveFrontLeft");
         encoderLeft = hardwareMap.dcMotor.get("collect");

         encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         encoderBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         encoderBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         //init wobble arm
         wobbleArmMotor = hardwareMap.dcMotor.get("wobbleArm");
         wobbleArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         wobbleArmMotor.setTargetPosition(0);
         wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         wobbleGrabServo = hardwareMap.servo.get("wobbleGrab");

         wobbleColorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
         wobbleTouchSensor = hardwareMap.get(RevTouchSensor.class, "touch");


         //init collection motor
         collectionMotor = hardwareMap.dcMotor.get("collect");
         collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         //init ring belt
         ringBeltMotor = hardwareMap.dcMotor.get("belt");
         ringBeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         ringBeltMotor .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         limitSwitch = hardwareMap.get(RevTouchSensor.class, "magLim");

         //init IMU
         imu  = hardwareMap.get(BNO055IMU.class, "imu");

         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

         parameters.mode = BNO055IMU.SensorMode.IMU;
         parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         parameters.loggingEnabled = false;

         imu.initialize(parameters);

         //Init Launch Motor
         DcMotor launcher = hardwareMap.dcMotor.get("launcher");

         MotorConfigurationType launchMotorType = launcher.getMotorType();
         launchMotorType.setGearing(3);
         launchMotorType.setTicksPerRev(84);
         launchMotorType.setMaxRPM(2400);

         launchMotor = launcher;
         launchMotor.setMotorType(launchMotorType);
         launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

         ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
     }

     public float getIMURotation() {
         return imu.getAngularOrientation().firstAngle;
     }

     //Unit conversions
     public double ticksToInches(double ticks) {
         return ticks * (ENCODER_CIRCUMFERENCE / COUNTS_PER_REVOLUTION);
     }

     public double inchesToTicks(double inches) {
         return inches * (COUNTS_PER_REVOLUTION / ENCODER_CIRCUMFERENCE);

     }


 }
