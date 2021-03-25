package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.drive.BulkMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RingPickupPathGenerator;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;

import java.util.ArrayList;

/**
 * Use OpenCV for ring recognition and Roadrunner for Path planning to achieve auto drive and shoot
 */
public class AutoWobbleGoalPickUpTask implements RobotControl{

    public enum TaskMode{
        DRIVE,
        PICKUP
    }

    TaskMode taskMode;
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    BulkMecanumDrive drive;
    Point result;
    long pickupTime;
    //share ring field coordinates between different instances to merge positions
    Vector2D wobblePosition;
    //perspective transform object
    static Mat trans = null;

    public static double  ROBOT_WIDTH = 18.0;

    /**
     * constructor
     * @param robotHardware
     * @param robotProfile
     */

    public AutoWobbleGoalPickUpTask(RobotHardware robotHardware, RobotProfile robotProfile) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.drive = robotHardware.getMecanumDrive();
    }

    public String toString() {
        return "AutoWobbleGoalPickUpTask";
    }

    /**
     * initialize the perspective transform
     */
    void init() {
        int ROBOT_WIDTH = 44; //cm
        ArrayList<Point> imgList = new ArrayList<Point>();
        ArrayList<Point> fieldList = new ArrayList<Point>();

        imgList.add(new Point(25, 38));
        imgList.add(new Point(292, 46));
        imgList.add(new Point(279, 117));
        imgList.add(new Point(544, 81));

        fieldList.add(new Point(60 + ROBOT_WIDTH/2, 25));
        fieldList.add(new Point(60 + ROBOT_WIDTH/2, 0));
        fieldList.add(new Point(25 + ROBOT_WIDTH/2, 0));
        fieldList.add(new Point(37 + ROBOT_WIDTH/2, -17));

        Mat imgMat = Converters.vector_Point2f_to_Mat(imgList);
        Mat fieldMat = Converters.vector_Point2f_to_Mat(fieldList);
        trans = Imgproc.getPerspectiveTransform(imgMat, fieldMat);
    }

    /**
     * Utility function to translate from robot coordinate into field coordinate
     * @param currPose current robot position and heading
     * @param robotViewPt ring position robot coordinate
     * @return ring position field coordinate
     */
    Vector2d getFieldCoordinate(Pose2d currPose, Point robotViewPt) {
        double fieldX = currPose.getX() + (robotViewPt.x * Math.cos(currPose.getHeading()) - robotViewPt.y*Math.sin(currPose.getHeading()))/2.54;
        double fieldY = currPose.getY() + (robotViewPt.x * Math.sin(currPose.getHeading()) + robotViewPt.y * Math.cos(currPose.getHeading()))/2.54;
        return new Vector2d(fieldX, fieldY);
    }

    /**
     * prepare stage
     */
    @Override
    public void prepare() {
        //initialize perspective transform when called for first time
        if(trans == null){
            init();
        }
        DriveConstraints constraints = new DriveConstraints(40.0, 30.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        RobotVision vision = robotHardware.getRobotVision();
        result = vision.getWobbleGoalHandle();
        Logger.logFile("AutoWobbleGoalPickUpTask: result is "+ result);
        Pose2d currPose;
        if (result!=null) {
            Mat transformed = new Mat();
            ArrayList<Point> imgPosList = new ArrayList<Point>();
            imgPosList.add(result);
            Core.perspectiveTransform(Converters.vector_Point2f_to_Mat(imgPosList), transformed, trans);
            ArrayList<Point> transPoints = new ArrayList<Point>();
            Converters.Mat_to_vector_Point2f(transformed, transPoints);
            for (Point p : transPoints) {
                Log.d("Point", p.toString());
                Logger.logFile("Wobble Goal robot coordinate at (cm) " + p.x + ", " + p.y);
            }
            currPose = robotHardware.getTrackingWheelLocalizer().getPoseEstimate();
            Vector2d worldCorr = getFieldCoordinate(currPose, transPoints.get(0));
            Logger.logFile("Current Pose " + currPose);
            Logger.logFile("Wobble Goal coordinate at (inch) " + worldCorr.getX() + ", " + worldCorr.getY());
            Vector2d endPose = new Vector2d(
                    worldCorr.getX()+robotProfile.hardwareSpec.wobbleOffsetX*Math.cos(currPose.getHeading()+Math.PI)
                            +robotProfile.hardwareSpec.wobbleOffsetY*Math.cos(currPose.getHeading()+Math.PI+Math.PI/2),
                    worldCorr.getY()+robotProfile.hardwareSpec.wobbleOffsetX*Math.sin(currPose.getHeading()+Math.PI)
                            +robotProfile.hardwareSpec.wobbleOffsetY*Math.sin(currPose.getHeading()+Math.PI+Math.PI/2));
            Logger.logFile("Wobble Goal Pick up coordinate at " + endPose);
            Trajectory trajectory = robotHardware.getMecanumDrive().trajectoryBuilder(currPose, constraints)
                                    .lineToConstantHeading(endPose)
                                    .build();
            robotHardware.setGrabberPosition(true);
            robotHardware.getMecanumDrive().followTrajectoryAsync(trajectory);
            taskMode = TaskMode.DRIVE;
        }
    }

    /**
     * when in picture mode: recognize ring position using OpenCV and translate into field coordinate
     * merge with existing ring array
     */
    @Override
    public void execute() {
        if(taskMode == TaskMode.DRIVE) {
            //DRIVE mode trigger Roadrunner movement
            drive.update();
            if (!drive.isBusy()) {
                taskMode = TaskMode.PICKUP;
                robotHardware.setArmMotorPos(RobotHardware.ArmPosition.GRAB);
                pickupTime= System.currentTimeMillis();
            }
        }
    }

    @Override
    public void cleanUp() {

    }

    /**
     * return done after half a second of pickup
     * @return
     */
    @Override
    public boolean isDone() {
        if(taskMode == TaskMode.PICKUP) {
            return pickupTime+200 < System.currentTimeMillis();
        }
        else {
            return result == null;
        }
    }
}
