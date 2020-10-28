package org.timecrafters.UltimateGoal;

import android.app.Activity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.timecrafters.TimeCraftersConfigurationTool.TimeCraftersConfiguration;

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

    private WebcamName webcam;
    private VuforiaLocalizer vuforia;

    //drive system
    public DcMotor encoderFront;
    public DcMotor encoderLeft;
    public DcMotor encoderBack;
    public DcMotor encoderRight;

    static final double BIAS_LEFT = -1.0;
    static final double BIAS_RIGHT = -0.87;

    //Conversion Constants
    static final double ENCODER_CIRCUMFERENCE = Math.PI * 4;
    static final int COUNTS_PER_REVOLUTION = 8192;
    static final float mmPerInch = 25.4f;

    // Inches Forward of axis of rotation
    static final float CAMERA_FORWARD_DISPLACEMENT  = 4.25f;
    // Inches above Ground
    static final float CAMERA_VERTICAL_DISPLACEMENT = 4.5f;
    // Inches Left of axis of rotation
    static final float CAMERA_LEFT_DISPLACEMENT     = 2f;

    //Robot Localizatoin
    private double locationX;
    private double locationY;
    private float rotation;

    public double visionX;
    public double visionY;


    public double traveledLeft;
    public double traveledRight;

    private int encoderFrontPrevious = 0;
    private int encoderLeftPrevious = 0;
    private int encoderBackPrevious = 0;
    private int encoderRightPrevious = 0;
    private float rotationPrevious = 0;

    //vuforia navigation
    public boolean trackableVisible;
    private VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> trackables = new ArrayList<VuforiaTrackable>();
    private OpenGLMatrix lastConfirmendLocation;

    //TensorFlow Object Detection
    public TFObjectDetector tfObjectDetector;
    private static final float MINIMUM_CONFIDENCE = 0.8f;


    public void initHardware() {
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        imu  = hardwareMap.get(BNO055IMU.class, "imu");
//        encoderFront = hardwareMap.dcMotor.get("encoderFront");
        encoderLeft = hardwareMap.dcMotor.get("encoderLeft");
//        encoderBack = hardwareMap.dcMotor.get("encoderBack");
        encoderRight = hardwareMap.dcMotor.get("encoderRight");

        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        encoderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        initVuforia();
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

    private void initTensorFlow() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters parameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        parameters.minResultConfidence = MINIMUM_CONFIDENCE;
        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(parameters, vuforia);
        tfObjectDetector.loadModelFromAsset("UltimateGoal.tflite", "Quad", "Single");
    }

    public void setDrivePower(double powerLeft, double powerRight){
        encoderLeft.setPower(powerLeft * BIAS_LEFT);
        encoderRight.setPower(powerRight * BIAS_RIGHT);

    }

    public void updateLocation(){
        // orientation is inverted to have clockwise be positive.
        float imuAngle = -imu.getAngularOrientation().firstAngle;

        float rotationChange = imuAngle - rotationPrevious;
        int encoderLeftCurrent = encoderLeft.getCurrentPosition();
        int encoderRightCurrent = encoderRight.getCurrentPosition();
        double encoderLeftChange = encoderLeftCurrent - encoderLeftPrevious;
        double encoderRightChange = encoderRightCurrent - encoderRightPrevious;

        traveledLeft += Math.abs(encoderLeftChange);
        traveledRight += Math.abs(encoderRightChange);
        rotation += rotationChange;

        encoderLeftPrevious = encoderLeftCurrent;
        encoderRightPrevious = encoderRightCurrent;
        rotationPrevious = imuAngle;

        double average = (encoderLeftChange+encoderRightChange)/2;

        double xChange = average * (Math.sin(Math.toRadians(rotation)));
        double yChange = average * (Math.cos(Math.toRadians(rotation)));

        locationX += xChange;
        locationY += yChange;

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
//                visionX = -translation.get(1) / mmPerInch;
//                visionY = translation.get(0) / mmPerInch;


                //For our tornament, it makes sence to make zero degrees towards the goal, orientation is inverted to have clockwise be positive.
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

    public double ticksToInches(double ticks) {
        return ticks * (ENCODER_CIRCUMFERENCE / COUNTS_PER_REVOLUTION);
    }

    public double inchesToTicks(double inches) {
        return inches * (COUNTS_PER_REVOLUTION / ENCODER_CIRCUMFERENCE);
    }

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

    public void deactivateVuforia() {

    }
}
