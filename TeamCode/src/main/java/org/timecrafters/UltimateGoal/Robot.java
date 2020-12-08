package org.timecrafters.UltimateGoal;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

public class   Robot {

    private HardwareMap hardwareMap;

    public Robot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public TimeCraftersConfiguration stateConfiguration = new TimeCraftersConfiguration();
    public BNO055IMU imu;

    private WebcamName webcam;
    private VuforiaLocalizer vuforia;

    //drive system

    public DcMotor driveBackLeft;
    public DcMotor driveFrontLeft;
    public DcMotor driveBackRight;
    public DcMotor driveFrontRight;

    static final double BIAS_FRONT_LEFT = 1;
    static final double BIAS_FRONT_RIGHT = 1;
    static final double BIAS_BACK_LEFT = 1;
    static final double BIAS_BACK_RIGHT = 1;

    static final double FINE_CORRECTION = 0.01;
    static final double LARGE_CORRECTION = 0.01 ;

    //Conversion Constants
    static final double ENCODER_CIRCUMFERENCE = Math.PI * 4;
    static final int COUNTS_PER_REVOLUTION = 8192;
    static final float mmPerInch = 25.4f;

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

    //Debugging
    public double visionX;
    public double visionY;
    public float rawAngle;
//    private String TestingRecord = "FrontLeft,FrontRight,BackLeft,BackRight";
    private String TestingRecord = "Rotation";

    public double traveledLeft;
    public double traveledRight;


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
//        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        imu  = hardwareMap.get(BNO055IMU.class, "imu");

        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        driveFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

//        initVuforia();

        rotation = stateConfiguration.variable("system", "startPos", "direction").value();
        locationX = stateConfiguration.variable("system", "startPos", "x").value();
        locationY = stateConfiguration.variable("system", "startPos", "y").value();
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
        parameters.minResultConfidence = MINIMUM_CONFIDENCE;
        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(parameters, vuforia);
        tfObjectDetector.loadModelFromAsset("UltimateGoal.tflite", "Quad", "Single");
    }

    //run this in every exec to track the robot's location.
    public void updateLocation(){
        // orientation is inverted to have clockwise be positive.
        float imuAngle = -imu.getAngularOrientation().firstAngle;
        double rotationChange = imuAngle - rotationPrevious;

        if (rotationChange > 180) {
            rotationChange -= 360;
        }
        if (rotationChange < -180) {
            rotationChange += 360;
        }

        int encoderLeftCurrent = driveFrontLeft.getCurrentPosition();
        int encoderRightCurrent = driveFrontRight.getCurrentPosition();
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
        driveFrontLeft.setPower(powerFrontLeft * BIAS_FRONT_LEFT);
        driveFrontRight.setPower(powerFrontRight * BIAS_FRONT_RIGHT);
        driveBackLeft.setPower(powerBackLeft * BIAS_BACK_LEFT);
        driveBackRight.setPower(powerBackRight * BIAS_BACK_RIGHT);
    }

    //returns an array of the powers necessary to execute the provided motion. "degreesDirectionMotion"
    //is the angle relative to the field that the robot should drive at. "degreesDirectionFace" is
    //the angle the robot should face relative to the field. The order of the output powers is
    //is ForwardLeft, ForwardRight, BackLeft, BackRight
    public double[] getMecanumPowers(float degreesDirectionMotion, double scalar, float degreesDirectionFace) {
        double rad = Math.toRadians(getRelativeAngle(degreesDirectionFace,degreesDirectionMotion));
        double y = scalar * Math.cos(rad);
        double x = scalar * Math.sin(rad);

        //TODO: Try swapping p and q if left and right seam incorrect
        double p = y + x;
        double q = y - x;

        float relativeRotation =  getRelativeAngle(degreesDirectionFace, rotation);
        double turnCorrection = Math.pow(LARGE_CORRECTION * relativeRotation, 3) + FINE_CORRECTION * relativeRotation;

        double powerForwardRight = q + turnCorrection;
        double powerForwardLeft = p - turnCorrection;
        double powerBackRight = p + turnCorrection;
        double powerBackLeft = q - turnCorrection;


        // The "extreme" is the power value that is furthest from zero. When this values exceed the
        // -1 to 1 power range, dividing the powers by the "extreme" scales everything back into the
        // workable range without altering the final motion vector;

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

    //
    public double[] getFacePowers(float direction, double power) {
        double relativeAngle = getRelativeAngle(direction, rotation);
        double scaler = Math.pow(LARGE_CORRECTION * relativeAngle, 3) + FINE_CORRECTION * relativeAngle;

        double left = -power * scaler;
        double right = power *scaler;


//        if (relativeAngle > 0) {
//            left *= -1;
//            right *= -1;
//        }
        double[] powers = {left,right};
        return powers;
    }

    //This function should not be used
    public void driveAtAngle(float angle, double power) {

        double relativeAngle = getRelativeAngle(angle, getRotation());

        //calculate how the power of each motor should be adjusted to make the robot curve
        //towards the target angle
        //--------------------------------------------------------------------------------------

        double turnPowerCorrection = Math.pow(0.03 * relativeAngle, 3) + 0.02 * relativeAngle;

        //Adjusts power based on degrees off from target.
        double leftPower = power - turnPowerCorrection;
        double rightPower = power + turnPowerCorrection;
        //--------------------------------------------------------------------------------------


        //calculates speed adjuster that slows the motors to be closer to the BasePower while
        // maintaining the power ratio nesesary to execute the turn.
        double powerAdjust = ((2 * power) / (Math.abs(leftPower) + Math.abs(rightPower)));

//        setDrivePower(leftPower * powerAdjust, rightPower * powerAdjust);
    }

    //Data Recording
//    public void record(double frontLeft, double frontRight, double backLeft, double backRight) {
//        TestingRecord+="\n"+frontLeft+","+frontRight+","+backLeft+","+backRight;
//    }

    public void record() {
        TestingRecord+="\n"+rotation;
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
