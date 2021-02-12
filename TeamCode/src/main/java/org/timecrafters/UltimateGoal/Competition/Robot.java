package org.timecrafters.UltimateGoal.Competition;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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

    public TimeCraftersConfiguration stateConfiguration = new TimeCraftersConfiguration();
    public BNO055IMU imu;

    //drive system
    public DcMotor driveFrontLeft;
    public DcMotor driveBackLeft;
    public DcMotor driveFrontRight;
    public DcMotor driveBackRight;

    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderBack;

    //Steering Constants
    static final double FINE_CORRECTION = 0.055 ;
    static final double LARGE_CORRECTION = 0.025;
    static final double MOMENTUM_CORRECTION = 1.045;
    static final double MOMENTUM_MAX_CORRECTION = 1.4;
    static final double MOMENTUM_HORIZONTAL_CORRECTION = -(Math.log10(MOMENTUM_MAX_CORRECTION-1)/Math.log10(MOMENTUM_CORRECTION));

    //Conversion Constants
    static final double ENCODER_CIRCUMFERENCE = Math.PI * 2.3622;
    static final int COUNTS_PER_REVOLUTION = 8192;
    static final float mmPerInch = 25.4f;
    static final double TICKS_PER_ROBOT_DEGREE_CLOCKWISE = 8.4;
    static final double TICKS_PER_ROBOT_DEGREE_COUNTERCLOCKWISE = 8.6;

    // Inches Forward of axis of rotation
    static final float CAMERA_FORWARD_DISPLACEMENT  = 13f;
    // Inches above Ground
    static final float CAMERA_VERTICAL_DISPLACEMENT = 4.5f;
    // Inches Left of axis of rotation
    static final float CAMERA_LEFT_DISPLACEMENT     = 2f;

    //Robot Localization
    public double locationX;
    public double locationY;
    private float rotation;

    private int encoderLeftPrevious = 0;
    private int encoderBackPrevious = 0;
    private int encoderRightPrevious = 0;
    private float rotationPrevious = 0;
    public float angularVelocity;

    //Launcher
    public DcMotor launchMotor;
    public static final double LAUNCH_POWER = 0.75;

    private static final long LAUNCH_ACCEL_TIME = 500;
    public double launchPositionX;
    public double launchPositionY;
    public float launchRotation;
    public static final double LAUNCH_TOLERANCE_POS = 0.5 * (COUNTS_PER_REVOLUTION/ENCODER_CIRCUMFERENCE);
    public static final double LAUNCH_TOLERANCE_FACE = 0.5;

    public boolean initLauncher;

    //Ring Intake
    public DcMotor collectionMotor;
    public Rev2mDistanceSensor ringIntakeSensor;

    public static final double RING_DETECT_DISTANCE = 100;
    public static final double RING_DETECT_DELAY = 1000;

    //Ring Belt
    public DcMotor ringBeltMotor;
    public RevTouchSensor limitSwitch;
    public int ringBeltStage;
    public static final int RING_BELT_LOOP_TICKS = 2544;
    public static final int RING_BELT_GAP = 670;

    public static final double RING_BELT_POWER = 0.2;

    //Wobble Goal Arm
    public DcMotor wobbleArmMotor;
    public Servo  wobbleGrabServo;
    public static final int WOBBLE_ARM_DOWN = -710;
    public static final double WOBBLE_SERVO_MAX = 0.3;
    public RevColorSensorV3 wobbleColorSensor;

    //vuforia navigation
    private WebcamName webcam;
    private VuforiaLocalizer vuforia;
    public Servo webCamServo;
    public static final double CAM_SERVO_UP = 0.2;
    public static final double CAM_SERVO_Down = 0.2;

    public boolean trackableVisible;
    private VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> trackables = new ArrayList<VuforiaTrackable>();
    private OpenGLMatrix lastConfirmendLocation;

    private long timeStartZeroVelocity = 0;
    private long minCheckDurationMs = 500;
    private int minCheckVelocity = 1;

    //TensorFlow Object Detection
    public TFObjectDetector tfObjectDetector;
    private static final float MINIMUM_CONFIDENCE = 0.8f;

    //Debugging
    public double totalV;
    public double visionX;
    public double visionY;
    public float rawAngle;
//    private String TestingRecord = "FrontLeft,FrontRight,BackLeft,BackRight";
    private String TestingRecord = "TicksPerDegree";

    public double traveledLeft;
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

        wobbleGrabServo = hardwareMap.servo.get("wobbleGrab");

        //init ring belt
        collectionMotor = hardwareMap.dcMotor.get("collect");
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //init ring belt
        ringBeltMotor = hardwareMap.dcMotor.get("belt");
        ringBeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ringBeltMotor .setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

        rotation = stateConfiguration.variable("system", "startPos", "direction").value();
        locationX = stateConfiguration.variable("system", "startPos", "x").value();
        locationY = stateConfiguration.variable("system", "startPos", "y").value();

        minCheckVelocity =stateConfiguration.variable("system", "tensorFlow", "minCheckV").value();
        minCheckDurationMs =stateConfiguration.variable("system", "tensorFlow", "minCheckMS").value();

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

        if (initLauncher) {
            double launcherPower = 0;
            long launchAccelStart = System.currentTimeMillis();
            while (launcherPower < LAUNCH_POWER) {
                launcherPower = (double) ((System.currentTimeMillis() - launchAccelStart) / LAUNCH_ACCEL_TIME) * LAUNCH_POWER;
                launchMotor.setPower(launcherPower);
            }
        }
        //
        launchPositionX = stateConfiguration.variable("system", "launchPos","x").value();
        launchPositionX = stateConfiguration.variable("system", "launchPos","y").value();
        launchRotation = stateConfiguration.variable("system", "launchPos","rot").value();

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
        parameters.minResultConfidence = stateConfiguration.variable("system", "tensorFlow", "minConfidence").value();
        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(parameters, vuforia);
        tfObjectDetector.loadModelFromAsset("UltimateGoal.tflite", "Quad", "Single");
    }

    //run this in every exec to track the robot's location.
    public void updateLocation(){

        // orientation is inverted to have clockwise be positive.
        float imuAngle = -imu.getAngularOrientation().firstAngle;
        double rotationChange = getRelativeAngle(rotationPrevious, imuAngle);

        int encoderLeftCurrent = -encoderLeft.getCurrentPosition();
        int encoderRightCurrent = -encoderRight.getCurrentPosition();
        int encoderBackCurrent = encoderBack.getCurrentPosition();

        double encoderLeftChange = encoderLeftCurrent - encoderLeftPrevious;
        double encoderRightChange = encoderRightCurrent - encoderRightPrevious;
        double encoderBackChange = encoderBackCurrent - encoderBackPrevious;

        encoderLeftPrevious = encoderLeftCurrent;
        encoderRightPrevious = encoderRightCurrent;
        encoderBackPrevious = encoderBackCurrent;
        rotationPrevious = imuAngle;

        //The forward Vector has the luxury of having an odometer on both sides of the robot.
        //This allows us to eliminate the unwanted influence of turning the robot by averaging
        //the two. This meathod doesn't require any prior calibration.
        double forwardVector = (encoderLeftChange+encoderRightChange)/2;

        //Since there isn't a second wheel to remove the influence of robot rotation, we have to
        //instead do this by approximating the number of ticks that were removed due to rotation
        //based on a separate calibration program.

        double ticksPerDegree;

        if (rotationChange < 0) {
            ticksPerDegree = TICKS_PER_ROBOT_DEGREE_COUNTERCLOCKWISE;
        } else {
            ticksPerDegree = TICKS_PER_ROBOT_DEGREE_CLOCKWISE;
        }

        double sidewaysVector = encoderBackChange + (rotationChange* ticksPerDegree);

        double magnitude = Math.sqrt((forwardVector*forwardVector) + (sidewaysVector*sidewaysVector));
        double direction = Math.toRadians(rotation + (rotationChange/2)) + Math.atan2(sidewaysVector,forwardVector);

        double xChange = magnitude * (Math.sin(direction));
        double yChange = magnitude * (Math.cos(direction));

        locationX += xChange;
        locationY += yChange;

        rotation += rotationChange;


        totalV = Math.abs(encoderLeftChange) + Math.abs(encoderRightChange) + Math.abs(encoderBackChange);


        if (totalV < minCheckVelocity) {
            long timeCurrent = System.currentTimeMillis();

            if (timeStartZeroVelocity == 0) {
                timeStartZeroVelocity = timeCurrent;
            } else if (timeCurrent - timeStartZeroVelocity >= minCheckDurationMs) {
                syncWithVuforia();
            }

        } else {
            timeStartZeroVelocity = 0;
        }


        if (rotation > 180) {
            rotation -= 360;
        }
        if (rotation < -180) {
            rotation += 360;
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


                VectorF translation = lastConfirmendLocation.getTranslation();
                locationX = -inchesToTicks(translation.get(1) / mmPerInch);
                locationY = inchesToTicks( translation.get(0) / mmPerInch);



                //For our tournament, it makes sense to make zero degrees towards the goal.
                //Orientation is inverted to have clockwise be positive.
                Orientation rotation = Orientation.getOrientation(lastConfirmendLocation, EXTRINSIC, XYZ, DEGREES);
                this.rotation = 90-rotation.thirdAngle;

                if (this.rotation > 180) {
                    this.rotation -= -180;
                }

                break;
            }
        }
    }

    public float getRotation() {
        return rotation;
    }

    public double getLocationX() {
        return locationX;
    }

    public double getLocationY() {
        return locationY;
    }

    //Manually set the position of the robot on the field.
    public void setCurrentPosition(float rotation, double x, double y) {
        this.rotation = rotation;
        locationX = x;
        locationY = y;
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
        float relativeRotation =  getRelativeAngle(degreesDirectionFace, rotation);
        double turnCorrection = Math.pow(LARGE_CORRECTION * relativeRotation, 3) + FINE_CORRECTION * relativeRotation;

        double powerForwardRight = scalar * (q + turnCorrection);
        double powerForwardLeft = scalar * (p - turnCorrection);
        double powerBackRight = scalar * (p + turnCorrection);
        double powerBackLeft = scalar * (q - turnCorrection);

//        //the turnCorrection often results in powers with magnitudes significantly larger than the
//        //scalar. The scaleRatio mitigates this without altering the quality of the motion by making
//        //it so that the average of the four magnitudes is equal to the scalar magnitude.
//        double powerSum = Math.abs(powerForwardRight) + Math.abs(powerForwardLeft) +
//                Math.abs(powerBackRight) + Math.abs(powerBackLeft);
//        double scaleRatio = (4 * Math.abs(scalar))/powerSum;
//
//        powerForwardRight *= scaleRatio;
//        powerForwardLeft *= scaleRatio;
//        powerBackRight *= scaleRatio;
//        powerBackLeft *= scaleRatio;


        if (relativeRotation != 0) {
            double momentumRelative =  angularVelocity * (relativeRotation / Math.abs(relativeRotation));
            double exponential = Math.pow(MOMENTUM_CORRECTION, MOMENTUM_HORIZONTAL_CORRECTION-momentumRelative);
            double momentumCorrection = (MOMENTUM_MAX_CORRECTION*exponential)/(1+exponential);
            powerForwardRight *= momentumCorrection;
            powerForwardLeft *= momentumCorrection;
            powerBackRight *= momentumCorrection;
            powerBackLeft *= momentumCorrection;
        }

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

//        double powerControlThreshold = 0.6 * ;
//
//        if (Math.min(extreme, 1) > powerControlThreshold) {
//            powerForwardLeft *= powerControlThreshold;
//            powerForwardRight *= powerControlThreshold;
//            powerBackLeft *= powerControlThreshold;
//            powerBackRight *= powerControlThreshold;
//        }

        double[] powers = {powerForwardLeft, powerForwardRight, powerBackLeft, powerBackRight};

        return powers;
    }

    //Outputs the power necessary to turn and face a provided direction
    public double[] getFacePowers(float direction, double power) {
        angularVelocity = imu.getAngularVelocity().xRotationRate;
        double relativeAngle = getRelativeAngle(direction, rotation);
        double scaler = Math.pow(LARGE_CORRECTION * relativeAngle, 3) + FINE_CORRECTION * relativeAngle;

        if (relativeAngle != 0) {
            double momentumRelative =  angularVelocity * (relativeAngle / Math.abs(relativeAngle));
            double exponential = Math.pow(MOMENTUM_CORRECTION, MOMENTUM_HORIZONTAL_CORRECTION-momentumRelative);
            double momentumCorrection = (MOMENTUM_MAX_CORRECTION*exponential)/(1+exponential);

            scaler *= momentumCorrection;
        }

        double left = -power * scaler;
        double right = power *scaler;

        double[] powers = {left,right};
        return powers;
    }

    public void ringBeltOn() {
        ringBeltMotor.setPower(RING_BELT_POWER);
    }

    public int getBeltPos(){
        return loopPos(ringBeltMotor.getCurrentPosition());
    }

    public int loopPos(int pos) {
        pos %= RING_BELT_LOOP_TICKS;
        return pos;
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
