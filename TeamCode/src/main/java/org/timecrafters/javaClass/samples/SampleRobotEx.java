 package org.timecrafters.javaClass.samples;

/*
The robot object contains all the hardware and functions that are used in both teleOp and
Autonomous. This includes drive functions, localization functions, shared constants, and a few
general calculations and debugging tools.
*/

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.timecrafters.TimeCraftersConfigurationTool.TimeCraftersConfiguration;
import org.timecrafters.UltimateGoal.Competition.Robot;

import java.util.ArrayList;
import java.util.List;

 public class SampleRobotEx {

     private HardwareMap hardwareMap;

     public SampleRobotEx(HardwareMap hardwareMap) {
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

     //Motion Constants
     static final double CUBIC_CORRECTION = 0.035;
     static final double FACE_CUBIC_CORRECTION = 0.025;
     static final double LINEAR_CORRECTION = 0.055;
     static final double FACE_MIN_CORRECTION = 0.2;
     static final double FACE_LINEAR_CORRECTION = 0.025;
     static final double MOMENTUM_CORRECTION = 1.05;
     static final double MOMENTUM_MAX_CORRECTION = 1.4;
     static final double MOMENTUM_HORIZONTAL_CORRECTION =
             -(Math.log10(MOMENTUM_MAX_CORRECTION-1) / Math.log10(MOMENTUM_CORRECTION));
     static final double FACE_MOMENTUM_MAX_CORRECTION = 1.1;
     static final double FACE_MOMENTUM_CORRECTION = 1.06;
     static final double FACE_MOMENTUM_HORIZONTAL_CORRECTION =
             -(Math.log10(FACE_MOMENTUM_MAX_CORRECTION-1) / Math.log10(FACE_MOMENTUM_CORRECTION));
     static final double ZERO_POWER_THRESHOLD = 0.25;

     //Unit Conversion Constants
     static final double ENCODER_CIRCUMFERENCE = Math.PI * 2.3622;
     static final int COUNTS_PER_REVOLUTION = 8192;
     static final float mmPerInch = 25.4f;
     static final double TICKS_PER_ROBOT_DEGREE_CLOCKWISE_FORWARD = 12.3;
     static final double TICKS_PER_ROBOT_DEGREE_COUNTERCLOCKWISE_FORWARD = 18.8;
     static final double TICKS_PER_ROBOT_DEGREE_CLOCKWISE = 8.4;
     static final double TICKS_PER_ROBOT_DEGREE_COUNTERCLOCKWISE = 8.6;

     //Robot Localization
     private static double locationX;
     private static double locationY;
     private static float rotation;

     private int encoderLeftPrevious = 0;
     private int encoderBackPrevious = 0;
     private int encoderRightPrevious = 0;
     private float rotationPrevious = 0;
     public float angularVelocity;

     public double forwardVector;
     public double sidewaysVector;


     //vuforia && tensorFlow Stuff
     private WebcamName webcam;
     private VuforiaLocalizer vuforia;

     // Inches Forward of axis of rotation
     static final float CAMERA_FORWARD_DISPLACEMENT  = 8f;
     // Inches above Ground
     static final float CAMERA_VERTICAL_DISPLACEMENT = 9.5f;
     // Inches Left of axis of rotation
     static final float CAMERA_LEFT_DISPLACEMENT = 4f;

     static final double CAMERA_DISPLACEMENT_MAG =
             Math.hypot(CAMERA_FORWARD_DISPLACEMENT,CAMERA_LEFT_DISPLACEMENT);
     static final float CAMERA_DISPLACEMENT_DIRECTION =
             (float) -Math.atan(CAMERA_LEFT_DISPLACEMENT/CAMERA_FORWARD_DISPLACEMENT);

     public boolean trackableVisible;
     private VuforiaTrackables targetsUltimateGoal;
     private List<VuforiaTrackable> trackables = new ArrayList<VuforiaTrackable>();
     private OpenGLMatrix lastConfirmendLocation;

     private long timeStartZeroVelocity = 0;
     private long minCheckDurationMs = 500;
     private int minCheckVelocity = 1;
     private float vuforiaRotationCull;

     //The servo mount for our camera allows us to look down for ideal TensorFlow and look up for
     //ideal Vuforia Navigation
     public Servo webCamServo;
     public static final double CAM_SERVO_DOWN = 0.15;

     //TensorFlow Object Detection
     public TFObjectDetector tfObjectDetector;
     private static final float MINIMUM_CONFIDENCE = 0.8f;

     //Launcher
     public DcMotor launchMotor;
     public static final double LAUNCH_POWER = 0.715;

     private static final long LAUNCH_ACCEL_TIME = 500;
     //These variables were originally going to be used in both autonomous and teleOp
     public double launchPositionX;
     public double launchPositionY;
     public float launchRotation;
     public int reduceLaunchPos;

     public boolean initLauncher;

     //Ring Intake
     public DcMotor collectionMotor;

     //Ring Belt
     public DcMotor ringBeltMotor;
     public RevTouchSensor limitSwitch;
     public int ringBeltStage;
     public int ringBeltGap = 700;
     public static final double RING_BELT_SLOW_POWER = 0.2;
     public static final double RING_BELT_NORMAL_POWER = 0.6;
     private int ringBeltPrev;
     public long beltMaxStopTime;
     public int beltReverseTicks;
     public int beltMaxStopTicks;

     //Wobble Goal Arm & Grabber
     public DcMotor  wobbleArmMotor;
     public Servo  wobbleGrabServo;
     public int wobbleDownPos;
     public int wobbleUpPos;
     public int wobbleDropPos;
     public static final double WOBBLE_SERVO_OPEN = 0;
     public static final double WOBBLE_SERVO_CLOSED = 1;
     public RevColorSensorV3 wobbleColorSensor;
     public double wobbleScoreX;
     public double wobbleScoreY;
     public RevTouchSensor wobbleTouchSensor;

     //Debugging
     public double totalV;
     private String TestingRecord = "Raw IMU, Delta, Saved";

     public double traveledForward = 0;
     public DcMotorEx motorAmpsTest;

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

         Velocity startVelocity = new Velocity();
         startVelocity.xVeloc = 0;
         startVelocity.yVeloc = 0;
         startVelocity.zVeloc = 0;

         Position startPosition = new Position();
         startPosition.x = 0;
         startPosition.y = 0;
         startPosition.z = 0;

         imu.startAccelerationIntegration(startPosition,startVelocity, 10);

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

     //Localization Function! This function is represented in a flow diagram, earlier in the
     //software section
     //run this in every exec to track the robot's location.
     public void updateLocation(){

         // orientation is inverted to have clockwise be positive.
         float imuAngle = -imu.getAngularOrientation().firstAngle;
         double rotationChange = getRelativeAngle(rotationPrevious, imuAngle);

         int encoderLeftCurrent = -encoderLeft.getCurrentPosition();
         int encoderRightCurrent = encoderRight.getCurrentPosition();
         int encoderBackCurrent = encoderBack.getCurrentPosition();

         double encoderLeftChange = encoderLeftCurrent - encoderLeftPrevious;
         double encoderRightChange = encoderRightCurrent - encoderRightPrevious;
         double encoderBackChange = encoderBackCurrent - encoderBackPrevious;

         encoderLeftPrevious = encoderLeftCurrent;
         encoderRightPrevious = encoderRightCurrent;
         encoderBackPrevious = encoderBackCurrent;
         rotationPrevious = imuAngle;

         //The forward Vector has the luxury of having an odometer on both sides of the robot.
         //This allows us to reduce the unwanted influence of turning the robot by averaging
         //the two. unfortunatly we the current positioning of the odometers

         //Since there isn't a second wheel to remove the influence of robot rotation, we have to
         //instead do this by approximating the number of ticks that were removed due to rotation
         //based on a separate calibration program.

         double ticksPerDegreeForward;
         double ticksPerDegreeSideways;

         if (rotationChange < 0) {
             ticksPerDegreeSideways = TICKS_PER_ROBOT_DEGREE_COUNTERCLOCKWISE;
             ticksPerDegreeForward = TICKS_PER_ROBOT_DEGREE_COUNTERCLOCKWISE_FORWARD;
         } else {
             ticksPerDegreeSideways = TICKS_PER_ROBOT_DEGREE_CLOCKWISE;
             ticksPerDegreeForward = TICKS_PER_ROBOT_DEGREE_CLOCKWISE_FORWARD;
         }

         forwardVector = ((encoderLeftChange+encoderRightChange)/2) -
                 (rotationChange* ticksPerDegreeForward);

         sidewaysVector = encoderBackChange + (rotationChange * ticksPerDegreeSideways);

         double magnitude = Math.sqrt((forwardVector*forwardVector) + (sidewaysVector*sidewaysVector));
         double direction = Math.toRadians(SampleRobotEx.rotation + (rotationChange/2)) +
                 Math.atan2(sidewaysVector,forwardVector);

         double xChange = magnitude * (Math.sin(direction));
         double yChange = magnitude * (Math.cos(direction));

         locationX += xChange;
         locationY += yChange;

         SampleRobotEx.rotation += rotationChange;
         SampleRobotEx.rotation = scaleAngleRange(SampleRobotEx.rotation);

     }

     public float getRotation() {
         return SampleRobotEx.rotation;
     }

     public double getLocationX() {
         return SampleRobotEx.locationX;
     }

     public double getLocationY() {
         return SampleRobotEx.locationY;
     }

     //This is meant to only be used to indicate starting positions and to reorient the robot.
     public void setLocalization(float rotation, double x, double y) {
         SampleRobotEx.rotation = rotation;
         SampleRobotEx.locationX = x;
         SampleRobotEx.locationY = y;
     }

     //returns the angle from the robot's current position to the given target position.
     public float getAngleToPosition (double x, double y) {
         double differenceX = x- getLocationX();
         double differenceY = y- getLocationY();
         double angle = Math.toDegrees(Math.atan2(differenceX, differenceY ));

         return (float) angle;
     }

     //Unit conversions
     public double ticksToInches(double ticks) {
         return ticks * (ENCODER_CIRCUMFERENCE / COUNTS_PER_REVOLUTION);
     }

     public double inchesToTicks(double inches) {
         return inches * (COUNTS_PER_REVOLUTION / ENCODER_CIRCUMFERENCE);

     }

     //Returns the shortest angle between two directions, with positive angles indicating that the
     // reference is to the right (clockwise) of the current. Negative angles indicate that the
     // reference is to the left.
     public float getRelativeAngle(float reference, float current) {
         return scaleAngleRange(current - reference);
     }

     //Drive Functions
     public void setDrivePower(double powerFrontLeft, double powerFrontRight,
                               double powerBackLeft, double powerBackRight){
         driveFrontLeft.setPower(powerFrontLeft);
         driveFrontRight.setPower(powerFrontRight);
         driveBackLeft.setPower(powerBackLeft);
         driveBackRight.setPower(powerBackRight);
     }

     public void setDrivePower(double[] powers){
         driveFrontLeft.setPower(powers[0]);
         driveFrontRight.setPower(powers[1]);
         driveBackLeft.setPower(powers[2]);
         driveBackRight.setPower(powers[3]);
     }

     //returns an array of the powers necessary to execute the provided motion.
     //"degreesDirectionMotion" is the angle relative to the field that the robot should drive at.
     //"degreesDirectionFace" is the angle the robot should face relative to the field. The order of
     //the output powers is ForwardLeft, ForwardRight, BackLeft, BackRight
     public double[] getMecanumPowers(float degreesDirectionMotion, double scalar,
                                      float degreesDirectionFace) {
         angularVelocity = imu.getAngularVelocity().xRotationRate;

         //calculating the base mecanum powers so that the robot drives along the degreesDirectionMotion
         //once it is pointed towards the degreesDirectionFace
         double rad = Math.toRadians(getRelativeAngle(degreesDirectionFace,degreesDirectionMotion));
         double y = Math.cos(rad);
         double x = Math.sin(rad);

         double p = y + x;
         double q = y - x;



         //calculating correction needed to steer the robot towards the degreesDirectionFace
         float relativeRotation =
                 getRelativeAngle(degreesDirectionFace, SampleRobotEx.rotation);
         double turnCorrection =
                 Math.pow(CUBIC_CORRECTION * relativeRotation, 3) +
                         LINEAR_CORRECTION * relativeRotation;

         if (relativeRotation != 0) {
             double momentumRelative =  angularVelocity *
                     (relativeRotation / Math.abs(relativeRotation));
             double exponential =
                     Math.pow(MOMENTUM_CORRECTION, MOMENTUM_HORIZONTAL_CORRECTION-momentumRelative);
             double momentumCorrection = (MOMENTUM_MAX_CORRECTION*exponential)/(1+exponential);
             //reduces concern for  momentum when the angle is far away from target
             turnCorrection *= momentumCorrection + ((Math.abs(relativeRotation) *
                     (1  - momentumCorrection)) / 180 );
         }

         double powerForwardRight = scalar * (q + turnCorrection);
         double powerForwardLeft = scalar * (p - turnCorrection);
         double powerBackRight = scalar * (p + turnCorrection);
         double powerBackLeft = scalar * (q - turnCorrection);


         // The "extreme" is the power value that is furthest from zero. When this values exceed the
         // -1 to 1 power range, dividing the powers by the "extreme" scales everything back into the
         // workable range without altering the final motion vector.

         double extreme = Math.max(
                 Math.max(Math.abs(powerForwardRight),Math.abs(powerForwardLeft)),
                 Math.max(Math.abs(powerBackRight),Math.abs(powerBackLeft)));

         if (extreme > 1) {
             powerForwardRight = powerForwardRight/extreme;
             powerForwardLeft = powerForwardLeft/extreme;
             powerBackRight = powerBackRight/extreme;
             powerBackLeft = powerBackLeft/extreme;
         }

         double[] powers = {powerForwardLeft, powerForwardRight, powerBackLeft, powerBackRight};

         double totalPower = Math.abs(powerForwardLeft) +
                 Math.abs(powerForwardRight) +
                 Math.abs(powerBackLeft) +
                 Math.abs(powerBackRight);
         if (totalPower < ZERO_POWER_THRESHOLD) {
             powers = new double[] {0,0,0,0};
         }


         return powers;
     }
     //Outputs the power necessary to turn and face a provided direction
     public double[] getFacePowers(float direction, double power) {
         angularVelocity = imu.getAngularVelocity().xRotationRate;
         double relativeAngle = getRelativeAngle(direction, SampleRobotEx.rotation);
         double scaler = Math.pow(FACE_CUBIC_CORRECTION * relativeAngle, 3) +
                 FACE_LINEAR_CORRECTION * relativeAngle;

         if (relativeAngle > 0.5) {
             scaler += FACE_MIN_CORRECTION;
         } else if (relativeAngle < -0.5) {
             scaler -= FACE_MIN_CORRECTION;
         }

         if (relativeAngle != 0) {
             double momentumRelative =  angularVelocity * (relativeAngle / Math.abs(relativeAngle));
             double exponential = Math.pow(FACE_MOMENTUM_CORRECTION,
                     FACE_MOMENTUM_HORIZONTAL_CORRECTION-momentumRelative);
             double momentumCorrection = (MOMENTUM_MAX_CORRECTION*exponential)/(1+exponential);

             scaler *= momentumCorrection;
         }

         double left = -power * scaler;
         double right = power *scaler;

         double[] powers = {left,right};

         double totalPower = 2 * (Math.abs(left) + Math.abs(right));
         if (totalPower < ZERO_POWER_THRESHOLD) {
             powers = new double[] {0,0};
         }

         return powers;
     }

     public float scaleAngleRange(float angle) {
         if (angle < -180) {
             angle += 360;
         }
         if (angle > 180) {
             angle -= 360;
         }
         return angle;
     }

 }
