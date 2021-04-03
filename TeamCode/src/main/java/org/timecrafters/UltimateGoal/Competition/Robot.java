package org.timecrafters.UltimateGoal.Competition;

/*
The robot object contains all the hardware and functions that are used in both teleOp and
Autonomous. This includes drive functions, localization functions, shared constants, and a few
general calculations and debugging tools.
*/

import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

public class Robot {

    private HardwareMap hardwareMap;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    //The TimeCraftersConfiguration is part of a debugging and tuning tool that allows us to edit
    //variables saved on the phone, without having to re-download the whole program.
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
    static final double MOMENTUM_HORIZONTAL_CORRECTION = -(Math.log10(MOMENTUM_MAX_CORRECTION-1)/Math.log10(MOMENTUM_CORRECTION));
    static final double FACE_MOMENTUM_MAX_CORRECTION = 1.1;
    static final double FACE_MOMENTUM_CORRECTION = 1.06;
    static final double FACE_MOMENTUM_HORIZONTAL_CORRECTION = -(Math.log10(FACE_MOMENTUM_MAX_CORRECTION-1)/Math.log10(FACE_MOMENTUM_CORRECTION));

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

    //vuforia navigation
    private WebcamName webcam;
    private VuforiaLocalizer vuforia;

    // Inches Forward of axis of rotation
    static final float CAMERA_FORWARD_DISPLACEMENT  = 8f;
    // Inches above Ground
    static final float CAMERA_VERTICAL_DISPLACEMENT = 9.5f;
    // Inches Left of axis of rotation
    static final float CAMERA_LEFT_DISPLACEMENT = 4f;

    static final double CAMERA_DISPLACEMENT_MAG = Math.hypot(CAMERA_FORWARD_DISPLACEMENT,CAMERA_LEFT_DISPLACEMENT);
    static final float CAMERA_DISPLACEMENT_DIRECTION = (float) -Math.atan(CAMERA_LEFT_DISPLACEMENT/CAMERA_FORWARD_DISPLACEMENT);

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
    public double visionX;
    public double visionY;
    public double visionZ;
    public float rawAngle;
    private String TestingRecord = "x,y";

    public double forwardVector;
    public double sidewaysVector;

    public double traveledForward = 0;
    public double traveledRight;

    public void initHardware() {

        limitSwitch = hardwareMap.get(RevTouchSensor.class, "magLim");

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

        wobbleUpPos = stateConfiguration.variable("system","arm", "up").value();
        wobbleDownPos = stateConfiguration.variable("system","arm", "down").value();
        wobbleDropPos = stateConfiguration.variable("system","arm", "drop").value();

        wobbleGrabServo = hardwareMap.servo.get("wobbleGrab");

        wobbleColorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        wobbleTouchSensor = hardwareMap.get(RevTouchSensor.class, "touch");


        //init ring belt
        collectionMotor = hardwareMap.dcMotor.get("collect");
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //init ring belt
        ringBeltMotor = hardwareMap.dcMotor.get("belt");
        ringBeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ringBeltMotor .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beltMaxStopTime = stateConfiguration.variable("system","belt", "maxStopTime").value();
        beltMaxStopTicks = stateConfiguration.variable("system","belt", "maxStopTicks").value();
        beltReverseTicks = stateConfiguration.variable("system","belt", "reverseTicks").value();
        ringBeltGap = stateConfiguration.variable("system","belt","gap").value();

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

        //Init Localization
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        initVuforia();
        webCamServo = hardwareMap.servo.get("look");
        webCamServo.setDirection(Servo.Direction.REVERSE );

        minCheckVelocity =stateConfiguration.variable("system", "camera", "minCheckV").value();
        vuforiaRotationCull = stateConfiguration.variable("system", "camera", "rCull").value();
        minCheckDurationMs =stateConfiguration.variable("system", "camera", "minCheckMS").value();

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

        initLauncher = stateConfiguration.action("system","initLauncher").enabled;
        reduceLaunchPos = stateConfiguration.variable("system", "launchPos", "reducePower").value();

        if (initLauncher) {
            double launcherPower = 0;
            long launchAccelStart = System.currentTimeMillis();
            while (launcherPower < LAUNCH_POWER) {
                launcherPower = (double) ((System.currentTimeMillis() - launchAccelStart) / LAUNCH_ACCEL_TIME) * LAUNCH_POWER;
                launchMotor.setPower(launcherPower);
            }
        }
        //
        launchPositionX = inchesToTicks((double) stateConfiguration.variable("system", "launchPos","x").value());
        launchPositionY = inchesToTicks((double) stateConfiguration.variable("system", "launchPos","y").value());
        launchRotation = stateConfiguration.variable("system", "launchPos","rot").value();

        initTensorFlow();

        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
    }

    private void initVuforia() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Abmu1jv/////AAABmYzrcgDEi014nv+wD6PkEPVnOlV2pI3S9sGUMMR/X7hF72x20rP1JcVtsU0nI6VK0yUlYbCSA2k+yMo4hQmPDBvrqeqAgXKa57ilPhW5e1cB3BEevP+9VoJ9QYFhKA3JJTiuFS50WQeuFy3dp0gOPoqHL3XClRFZWbhzihyNnLXgXlKiq+i5GbfONECucQU2DgiuuxYlCaeNdUHl1X5C2pO80zZ6y7PYAp3p0ciXJxqfBoVAklhd69avaAE5Z84ctKscvcbxCS16lq81X7XgIFjshLoD/vpWa300llDG83+Y777q7b5v7gsUCZ6FiuK152Rd272HLuBRhoTXAt0ug9Baq5cz3sn0sAIEzSHX1nah";
        parameters.cameraName = webcam;
        parameters.useExtendedTracking = false;


        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        trackables.addAll(targetsUltimateGoal);

        final float mmTargetHeight   = (6) * mmPerInch;

        final float halfField = 72 * mmPerInch;
        final float quadField  = 36 * mmPerInch;

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT * mmPerInch, CAMERA_LEFT_DISPLACEMENT * mmPerInch, CAMERA_VERTICAL_DISPLACEMENT * mmPerInch)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, -90, 0, 0));

        for (VuforiaTrackable trackable : trackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsUltimateGoal.activate();
    }

    public void deactivateVuforia() {
        targetsUltimateGoal.deactivate();
    }

    private void initTensorFlow() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters parameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        parameters.minResultConfidence = stateConfiguration.variable("system", "camera", "minConfidence").value();
        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(parameters, vuforia);
        tfObjectDetector.loadModelFromAsset("UltimateGoal.tflite", "Quad", "Single");
    }

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

        forwardVector = ((encoderLeftChange+encoderRightChange)/2) -  (rotationChange* ticksPerDegreeForward);

        traveledForward += forwardVector;
        sidewaysVector = encoderBackChange + (rotationChange * ticksPerDegreeSideways);

        double magnitude = Math.sqrt((forwardVector*forwardVector) + (sidewaysVector*sidewaysVector));
        double direction = Math.toRadians(Robot.rotation + (rotationChange/2)) + Math.atan2(sidewaysVector,forwardVector);

        double xChange = magnitude * (Math.sin(direction));
        double yChange = magnitude * (Math.cos(direction));

        locationX += xChange;
        locationY += yChange;

        Robot.rotation += rotationChange;


        totalV = Math.abs(encoderLeftChange) + Math.abs(encoderRightChange) + Math.abs(encoderBackChange);

        if (Robot.rotation > 180) {
            Robot.rotation -= 360;
        }
        if (Robot.rotation < -180) {
            Robot.rotation += 360;
        }

    }

    public void syncIfStationary() {
        if (totalV < minCheckVelocity) {
            long timeCurrent = System.currentTimeMillis();

            if (timeStartZeroVelocity == 0) {
                timeStartZeroVelocity = timeCurrent;
            } else if (timeCurrent - timeStartZeroVelocity >= minCheckDurationMs ) {
                syncWithVuforia();
            }

        } else {
            timeStartZeroVelocity = 0;
        }

    }

    public void syncWithVuforia() {
        trackableVisible = false;
        for (VuforiaTrackable trackable : trackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocation = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();

                //this is used for debugging purposes.
                trackableVisible = true;

                if (robotLocation != null) {
                    lastConfirmendLocation = robotLocation;
                }

                //For our tournament, it makes sense to make zero degrees towards the goal.
                //Orientation is inverted to have clockwise be positive.
                Orientation orientation = Orientation.getOrientation(lastConfirmendLocation, EXTRINSIC, XYZ, DEGREES);
                float vuforiaRotation = 90-orientation.thirdAngle;

                if (vuforiaRotation > 180) {
                    vuforiaRotation -= -180;
                }

                if (Math.abs(rotation - vuforiaRotation) < vuforiaRotationCull) {
                    rotation = vuforiaRotation;

                    VectorF translation = lastConfirmendLocation.getTranslation();
                    double camX = -translation.get(1) / mmPerInch;
                    double camY = translation.get(0) / mmPerInch;

                    double displaceX = CAMERA_DISPLACEMENT_MAG * Math.sin(Robot.rotation + 180 - CAMERA_DISPLACEMENT_DIRECTION);
                    double displaceY = CAMERA_DISPLACEMENT_MAG * Math.cos(Robot.rotation + 180 - CAMERA_DISPLACEMENT_DIRECTION);

                    locationX = inchesToTicks(camX - displaceX);
                    locationY = inchesToTicks(camY - displaceY);

                    break;
                }
            }
        }
    }

    public float getRotation() {
        return Robot.rotation;
    }

    public double getLocationX() {
        return Robot.locationX;
    }

    public double getLocationY() {
        return Robot.locationY;
    }

    public void setLocalization(float rotation, double x, double y) {
        Robot.rotation = rotation;
        Robot.locationX = x;
        Robot.locationY = y;
    }

    //Manually set the position of the robot on the field.
    public void setCurrentPosition(float rotation, double x, double y) {
        Robot.rotation = rotation;
        Robot.locationX = x;
        Robot.locationY = y;
    }

    //returns the angle from the robot's current position to the given target position.
    public float getAngleToPosition (double x, double y) {
        double differenceX = x- getLocationX();
        double differenceY = y- getLocationY();
        double angle = Math.toDegrees(Math.atan2(differenceX, differenceY ));

        return (float) angle;

    }

    //Unit conversion
    public double ticksToInches(double ticks) {
        return ticks * (ENCODER_CIRCUMFERENCE / COUNTS_PER_REVOLUTION);
    }

    public double inchesToTicks(double inches) {
        return inches * (COUNTS_PER_REVOLUTION / ENCODER_CIRCUMFERENCE);

    }

    //Returns the angle between two angles, with positive angles indicating that the reference is
    //to the right (clockwise) of the current. Negative angles indicate that the reference is to the
    //left.
    public float getRelativeAngle(float reference, float current) {
        float relative = current - reference;

        if (relative < -180) {
            relative += 360;
        }

        if (relative > 180) {
            relative -= 360;
        }
        return relative;
    }

    //Drive Functions
    public void setDrivePower(double powerFrontLeft, double powerFrontRight, double powerBackLeft, double powerBackRight){
        driveFrontLeft.setPower(powerFrontLeft);
        driveFrontRight.setPower(powerFrontRight);
        driveBackLeft.setPower(powerBackLeft);
        driveBackRight.setPower(powerBackRight);
    }

    //returns an array of the powers necessary to execute the provided motion. "degreesDirectionMotion"
    //is the angle relative to the field that the robot should drive at. "degreesDirectionFace" is
    //the angle the robot should face relative to the field. The order of the output powers is
    //is ForwardLeft, ForwardRight, BackLeft, BackRight
    public double[] getMecanumPowers(float degreesDirectionMotion, double scalar, float degreesDirectionFace) {
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
                getRelativeAngle(degreesDirectionFace, Robot.rotation);
        double turnCorrection =
                Math.pow(CUBIC_CORRECTION * relativeRotation, 3) +
                        LINEAR_CORRECTION * relativeRotation;

        if (relativeRotation != 0) {
            double momentumRelative =  angularVelocity * (relativeRotation / Math.abs(relativeRotation));
            double exponential = Math.pow(MOMENTUM_CORRECTION, MOMENTUM_HORIZONTAL_CORRECTION-momentumRelative);
            double momentumCorrection = (MOMENTUM_MAX_CORRECTION*exponential)/(1+exponential);
            //reduces concern for  momentum when the angle is far away from target
            turnCorrection *= momentumCorrection + ((Math.abs(relativeRotation) * (1  - momentumCorrection)) / 180 );
//            turnCorrection *= momentumCorrection;
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

        return powers;
    }

    //Outputs the power necessary to turn and face a provided direction
    public double[] getFacePowers(float direction, double power) {
        angularVelocity = imu.getAngularVelocity().xRotationRate;
        double relativeAngle = getRelativeAngle(direction, Robot.rotation);
        double scaler = Math.pow(FACE_CUBIC_CORRECTION * relativeAngle, 3) + FACE_LINEAR_CORRECTION * relativeAngle;

        if (relativeAngle > 0.5) {
            scaler += FACE_MIN_CORRECTION;
        } else if (relativeAngle < -0.5) {
            scaler -= FACE_MIN_CORRECTION;
        }

        if (relativeAngle != 0) {
            double momentumRelative =  angularVelocity * (relativeAngle / Math.abs(relativeAngle));
            double exponential = Math.pow(FACE_MOMENTUM_CORRECTION, FACE_MOMENTUM_HORIZONTAL_CORRECTION-momentumRelative);
            double momentumCorrection = (MOMENTUM_MAX_CORRECTION*exponential)/(1+exponential);

            scaler *= momentumCorrection;
        }

        double left = -power * scaler;
        double right = power *scaler;

        double[] powers = {left,right};
        return powers;
    }

    public boolean beltIsStuck() {
        int ringBeltPos = ringBeltMotor.getCurrentPosition();
        boolean notMoved = (ringBeltPos - ringBeltPrev <= beltMaxStopTicks);
        ringBeltPrev = ringBeltPos;
        return notMoved;
    }

    public void record(String record) {
        TestingRecord+="\n"+record;
    }

    public void saveRecording() {
        writeToFile(Environment.getExternalStorageDirectory().getAbsolutePath()+File.separator+"TimeCrafters_TestingRecord"+File.separator+"RobotTestingRecord.txt", TestingRecord);
    }

    public boolean writeToFile(String filePath, String content) {
        try {

            FileWriter writer = new FileWriter(filePath);
            writer.write(content);
            writer.close();

            return true;

        } catch (IOException e) {
            Log.e("RecordTest", e.getLocalizedMessage());
            return false;
        }
    }


}
