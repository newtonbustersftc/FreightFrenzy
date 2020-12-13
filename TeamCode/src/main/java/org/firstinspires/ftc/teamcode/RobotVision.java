package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.study.VuforiaOpenCV;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 2020.11.05
 claire
 **/

public class RobotVision {
    public enum AutonomousGoal {NONE, SINGLE, QUAD};
    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackables targetsUltimateGoal;
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;


    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "AYqWjPr/////AAABmVMnC//AfkUUkkIWPJCncdsd7jeXMOENyhdSo1YOFElDrSAX3ZjibeHioHvx6055Ou8XzrLlhT/hQI2RrcVJpNwu/4xvxHWJPuPBcObXHLy47ami7MaV9rOIInppWCsnx33TqjSocCCscnGTclFAbbJRomvczCo9cLB63jZ77kReVF7VNX4GmOxyNx3xsHCI79EptzqpJDIEoGcL87I4u58lszEZ6JpTW1AEkb927VmmeDPpmo9st/+1G4I4IvudLtdqAQX5Pvepu5m8wXClOE4SWW09HK1qENij6IbGjaPS+HK4Go0dcSeFR4nCH/QQZos66R4cSRE1mrqBGsmsVqwU2Jcm7YSEvZwTWCk6M6Pz";

    private TFObjectDetector tfod;

    RobotHardware robotHardware;
    HardwareMap hardwareMap;

    CVPipeline pipeline = new CVPipeline();

    public void init(HardwareMap hardwareMap, RobotHardware robotHardware, RobotProfile robotProfile){
        Logger.logFile("RobotVision init()");
        this.hardwareMap = hardwareMap;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        initVuforia();
        MASK_LOWER_BOUND_H = robotProfile.cvParam.maskLowerH;
        MASK_LOWER_BOUND_S = robotProfile.cvParam.maskLowerS;
        MASK_LOWER_BOUND_V = robotProfile.cvParam.maskLowerV;
        MASK_UPPER_BOUND_H = robotProfile.cvParam.maskUpperH;
        MASK_UPPER_BOUND_S = robotProfile.cvParam.maskUpperS;
        MASK_UPPER_BOUND_V = robotProfile.cvParam.maskUpperV;
        CROP_TOP_PERCENT = robotProfile.cvParam.cropTop;
        MIN_AREA = robotProfile.cvParam.minArea;
     }

    private void initVuforia() {
        Logger.logFile("initVuforia " + this);
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Set RGB format for OpenCV to use
        // Set queue capacity to 1 so OpenCV can grab the latest frame
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        Logger.logFile("Vuforia Initialized");
    }
/*
    private void initTfod() {
        Logger.logFile("initTfod()");
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        Logger.logFile("Tfod Initialized");
    }
*/
    public Pose2d getNavigationLocalization() {
        double x, y, heading;
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            x = translation.get(0) / mmPerInch;
            y = translation.get(1) / mmPerInch;

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, AngleUnit.RADIANS);
            heading = rotation.thirdAngle;
            return new Pose2d(new Vector2d(x, y), heading-Math.PI/2);
        }
        else {
            return null;
        }
    }

    public void activateNavigationTarget(){
        Logger.logFile("activateNavTarget " + this + " targets " + targetsUltimateGoal);
        targetsUltimateGoal.activate();
    }

    public void deactivateNavigationTarget(){
        Logger.logFile("deactivate navigation " + this + " targets " + targetsUltimateGoal);
        targetsUltimateGoal.deactivate();
    }
/*
    public void activateRecognition(){
        Logger.logFile("activateRecognition " + this + " tfod " + tfod);
        tfod.activate();
        tfod.setZoom(2, 1.33);
    }

    public void deactivateRecognition(){
        Logger.logFile("deactivate recognition " + this + " tfod " + tfod);
        tfod.deactivate();
    }

    public List<Recognition> getRingRecognition(){
        return tfod.getRecognitions();
    }
*/
    public AutonomousGoal getAutonomousRecognition(){
        processCV();
        ArrayList<Rect> rects = pipeline.getRingRecList();
        if(rects.size() == 0) {
            return AutonomousGoal.NONE;
        }
        Rect r = rects.get(0);
        if(r.height > 50){
            return AutonomousGoal.QUAD;
        }
        else {
            return AutonomousGoal.SINGLE;
        }

    }

    private void processCV() {
    try {
        Image rgb = null;
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        /*rgb is now the Image object that weve used in the video*/
        if (rgb!=null) {
            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgb.getPixels());

            //put the image into a MAT for OpenCV
            Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
            Utils.bitmapToMat(bm, tmp);
            pipeline.processFrame(tmp);
        }
        //close the frame, prevents memory leaks and crashing
        frame.close();
    }
    catch (InterruptedException ex) {
    }
}

    public static int MASK_LOWER_BOUND_H = 20;
    public static int MASK_LOWER_BOUND_S = 150;
    public static int MASK_LOWER_BOUND_V = 100;
    public static int MASK_UPPER_BOUND_H = 30;
    public static int MASK_UPPER_BOUND_S = 255;
    public static int MASK_UPPER_BOUND_V = 255;
    public static int MIN_AREA = 5;
    public static int DIM_MULTIPLIER = 4;   // because we pyrDown twice, each with default factor of 2
    public static int CROP_TOP_PERCENT = 20;
    public static int CROP_LEFT_PERCENT = 0;
    public static int CROP_RIGHT_PERCENT = 0;
    public static int CROP_BOTTOM_PERCENT = 0;
    static Scalar DRAW_COLOR = new Scalar(255, 0, 0);
    static boolean picSaved = false;

    public static class CVPipeline extends OpenCvPipeline {
        int count;

        // Cache
        Mat procMat;
        Mat pyrDownMat = new Mat();
        Mat hsvMat = new Mat();
        Mat maskMat = new Mat();
        Mat dilatedMask = new Mat();
        Mat hierarchey = new Mat();
        ArrayList<Rect> ringRecList = new ArrayList<Rect>();

        @Override
        public Mat processFrame(Mat input) {
            int offsetX;
            int offsetY;
            if (CROP_TOP_PERCENT!=0 || CROP_BOTTOM_PERCENT!=0 || CROP_RIGHT_PERCENT!=0 || CROP_LEFT_PERCENT!=0) {
                offsetX = input.width()*CROP_LEFT_PERCENT/100;
                offsetY = input.height()*CROP_TOP_PERCENT/100;
                procMat = input.submat(new Rect(offsetX, offsetY,
                        input.width()*(100-CROP_LEFT_PERCENT-CROP_RIGHT_PERCENT)/100,
                        input.height()*(100-CROP_TOP_PERCENT-CROP_BOTTOM_PERCENT)/100));
            }
            else {
                offsetX = 0;
                offsetY = 0;
                procMat = input;
            }
            Log.i("OpenCV", "Offset:" + offsetX + "," + offsetY);
            Log.i("OpenCV", "CropImg:" + procMat.width() + "," + procMat.height());
            Imgproc.pyrDown(procMat, pyrDownMat);
            Imgproc.pyrDown(pyrDownMat, pyrDownMat);

            Imgproc.cvtColor(pyrDownMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
            Scalar lowerBound = new Scalar(MASK_LOWER_BOUND_H, MASK_LOWER_BOUND_S, MASK_LOWER_BOUND_V);
            Scalar upperBound = new Scalar(MASK_UPPER_BOUND_H, MASK_UPPER_BOUND_S, MASK_UPPER_BOUND_V);
            Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
            Imgproc.dilate(maskMat, dilatedMask, new Mat());

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(dilatedMask, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Iterator<MatOfPoint> each = contours.iterator();
            ringRecList.clear();
            int ndx = 0;
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                if (area > MIN_AREA) {
                    Rect rec = Imgproc.boundingRect(wrapper);
                    rec.x = rec.x * DIM_MULTIPLIER + offsetX;
                    rec.y = rec.y * DIM_MULTIPLIER + offsetY;
                    rec.width *= DIM_MULTIPLIER;
                    rec.height *= DIM_MULTIPLIER;
                    ringRecList.add(rec);
                    //Rect drawRec = new Rect(rec.x*DIM_MULTIPLIER, rec.y*DIM_MULTIPLIER, rec.width*DIM_MULTIPLIER, rec.height*DIM_MULTIPLIER);
                    if (!picSaved) {    // update drawing only when saving the picture
                        Imgproc.rectangle(input, rec, DRAW_COLOR, 2);
                    }
                    Log.i("Area", "Ndx:" + ndx + " Area:" + area + " Rec:" + rec.x + "," + rec.y + "," + rec.width + "," + rec.height);
                    count++;
                }
                ndx++;
            }
            if (!picSaved) {
                //need to save pic to file
                String timestamp = new SimpleDateFormat("MMdd-HHmmss", Locale.US).format(new Date());
                Mat mbgr = new Mat();
                Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
                Imgcodecs.imwrite("/sdcard/FIRST/S" + timestamp + ".jpg", mbgr);
                mbgr.release();

                picSaved = true;
            }
            return input;
        }

        public ArrayList<Rect> getRingRecList() {
            return ringRecList;
        }
    }
}

