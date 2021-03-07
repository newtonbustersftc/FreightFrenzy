/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.study;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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
 * This 2020-2021 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
 * and Blue Alliance perimeter walls.
 * Two additional targets are placed on the perimeter wall, one in front of each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Config
@TeleOp(name="Vuforia OpenCV", group ="Concept")
public class VuforiaOpenCV extends LinearOpMode {

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
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
    private static final String VUFORIA_KEY = "AYqWjPr/////AAABmVMnC//AfkUUkkIWPJCncdsd7jeXMOENyhdSo1YOFElDrSAX3ZjibeHioHvx6055Ou8XzrLlhT/hQI2RrcVJpNwu/4xvxHWJPuPBcObXHLy47ami7MaV9rOIInppWCsnx33TqjSocCCscnGTclFAbbJRomvczCo9cLB63jZ77kReVF7VNX4GmOxyNx3xsHCI79EptzqpJDIEoGcL87I4u58lszEZ6JpTW1AEkb927VmmeDPpmo9st/+1G4I4IvudLtdqAQX5Pvepu5m8wXClOE4SWW09HK1qENij6IbGjaPS+HK4Go0dcSeFR4nCH/QQZos66R4cSRE1mrqBGsmsVqwU2Jcm7YSEvZwTWCk6M6Pz";

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
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    CVPipeline pipeline = new CVPipeline();

    @Override public void runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraName = webcamName;
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Set RGB format for OpenCV to use
        // Set queue capacity to 1 so OpenCV can grab the latest frame
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        waitForStart();

        picSaved = false;
        while (!isStopRequested()) {
            processCV();
        }
    }

    public void processCV() {
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
                pipeline.getCount();
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
                    //if (!picSaved) {    // update drawing only when saving the picture
                        //Imgproc.rectangle(input, rec, DRAW_COLOR, 2);
                    //}
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

        public int getCount() {
            Log.i("Count", "Count - " + ringRecList.size());
            return ringRecList.size();
        }
    }
}
