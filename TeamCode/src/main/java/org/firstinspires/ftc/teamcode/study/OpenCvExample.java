package org.firstinspires.ftc.teamcode.study;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@Config
@TeleOp
public class OpenCvExample extends LinearOpMode
{
    public static int MASK_LOWER_BOUND_H = 10;
    public static int MASK_LOWER_BOUND_S = 150;
    public static int MASK_LOWER_BOUND_V = 100;
    public static int MASK_UPPER_BOUND_H = 20;
    public static int MASK_UPPER_BOUND_S = 255;
    public static int MASK_UPPER_BOUND_V = 255;
    public static int MIN_AREA = 5;
    public static int DRAW_RATIO = 4;
    static Scalar DRAW_COLOR = new Scalar(255, 0, 0);

    OpenCvInternalCamera phoneCam;
    CVPipeline pipeline;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new CVPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Count", pipeline.getCount());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class CVPipeline extends OpenCvPipeline {
        int count;

        // Cache
        Mat pyrDownMat = new Mat();
        Mat hsvMat = new Mat();
        Mat maskMat = new Mat();
        Mat dilatedMask = new Mat();
        Mat hierarchey = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.pyrDown(input, pyrDownMat);
            Imgproc.pyrDown(pyrDownMat, pyrDownMat);

            Imgproc.cvtColor(pyrDownMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
            Scalar lowerBound = new Scalar(MASK_LOWER_BOUND_H, MASK_LOWER_BOUND_S, MASK_LOWER_BOUND_V);
            Scalar upperBound = new Scalar(MASK_UPPER_BOUND_H, MASK_UPPER_BOUND_S, MASK_UPPER_BOUND_V);
            Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
            Imgproc.dilate(maskMat, dilatedMask, new Mat());

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(dilatedMask, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Iterator<MatOfPoint> each = contours.iterator();
            count = 0;
            int ndx = 0;
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                if (area > MIN_AREA) {
                    Rect rec = Imgproc.boundingRect(wrapper);
                    Rect drawRec = new Rect(rec.x*DRAW_RATIO, rec.y*DRAW_RATIO, rec.width*DRAW_RATIO, rec.height*DRAW_RATIO);
                    Imgproc.rectangle(input, drawRec, DRAW_COLOR, 2);
                    //Imgproc.drawContours(input, contours, ndx, DRAW_COLOR, 2, Imgproc.LINE_8,
                    //        hierarchey, 2, new Point());
                    Log.i("Area", "Ndx:" + ndx + " Area:" + area + " Rec:" + rec.x + "," + rec.y + "," + rec.width + "," + rec.height);
                    count++;
                }
                ndx++;
            }
            return input;
        }

        public int getCount()
        {
            Log.i("Count", "Count - " + count);
            return count;
        }
    }
}
