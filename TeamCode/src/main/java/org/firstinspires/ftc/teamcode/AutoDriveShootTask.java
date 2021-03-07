package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

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
public class AutoDriveShootTask implements RobotControl{

    public enum TaskMode{
        FIRST_PIC,
        MORE_PIC,
        DRIVE
    }

    TaskMode taskMode;
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    BulkMecanumDrive drive;
    //share ring field coordinates between different instances to merge positions
    static ArrayList<Vector2d> rings = null;
    //perspective transform object
    static Mat trans = null;
    ArrayList<Trajectory> arrayTraj;
    boolean doneDriving;
    Pose2d endPose = null;
    DriveConstraints driveContraints = null;
    boolean saveImg = true;

    public static double  ROBOT_WIDTH = 18.0;

    /**
     * constructor
     * @param robotHardware
     * @param robotProfile
     * @param taskMode
     */
    public AutoDriveShootTask(RobotHardware robotHardware, RobotProfile robotProfile, TaskMode taskMode){
        this(robotHardware, robotProfile, taskMode, true);
    }

    public AutoDriveShootTask(RobotHardware robotHardware, RobotProfile robotProfile, TaskMode taskMode, boolean saveImg){
        this.robotHardware = robotHardware;
        this.taskMode = taskMode;
        this.robotProfile = robotProfile;
        this.drive = robotHardware.getMecanumDrive();
        this.saveImg = saveImg;
    }

    public void setEndPose(Pose2d endPose) {
        this.endPose = endPose;
    }

    public void setSaveImage(boolean saveImage) {
        this.saveImg = saveImage;
    }

    public void setDriveConstraints(DriveConstraints driveConstraints) {
        this.driveContraints = driveConstraints;
    }

    public String toString() {
        return "AutoDriveShoot " + taskMode;
    }

    /**
     * initialize the perspective transform
     */
    void init() {
        int ROBOT_CAMERA_OFFSET = 17; //cm
        ArrayList<Point> imgList = new ArrayList<Point>();
        ArrayList<Point> fieldList = new ArrayList<Point>();

        imgList.add(new Point(305, 121));
        imgList.add(new Point(40, 186));
        imgList.add(new Point(560, 188));
        imgList.add(new Point(293, 344));

        fieldList.add(new Point(180 + ROBOT_CAMERA_OFFSET, 0));
        fieldList.add(new Point(120 + ROBOT_CAMERA_OFFSET, 45));
        fieldList.add(new Point(120 + ROBOT_CAMERA_OFFSET, -45));
        fieldList.add(new Point(60 + ROBOT_CAMERA_OFFSET, 0));

        Mat imgMat = Converters.vector_Point2f_to_Mat(imgList);
        Mat fieldMat = Converters.vector_Point2f_to_Mat(fieldList);
        trans = Imgproc.getPerspectiveTransform(imgMat, fieldMat);

        rings = new ArrayList<Vector2d>();
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
        //clear ring array when taking the first picture
        if(taskMode == TaskMode.FIRST_PIC){
            rings.clear();
        }
        //select and generate the best path to collect rings using Roadrunner
        else if(taskMode == TaskMode.DRIVE){
            if (endPose==null) {
                endPose = robotProfile.getProfilePose("SHOOT-DRIVER");
            }
            RingPickupPathGenerator ringPickupPathGenerator = new RingPickupPathGenerator(robotHardware.getTrackingWheelLocalizer().getPoseEstimate(),
                    endPose);

            long start = System.currentTimeMillis();
            arrayTraj = ringPickupPathGenerator.generatePath(rings);
            Logger.logFile("Path Gen Time: " + (System.currentTimeMillis() - start));
            Trajectory traj = arrayTraj.remove(0);
            drive.followTrajectoryAsync(traj);
            doneDriving = false;
        }
    }

    /**
     * when in picture mode: recognize ring position using OpenCV and translate into field coordinate
     * merge with existing ring array
     */
    @Override
    public void execute() {
        if(taskMode == TaskMode.MORE_PIC || taskMode == TaskMode.FIRST_PIC) {
            ArrayList<Rect> ringRecArray = robotHardware.getRobotVision().getRings(saveImg);
            Logger.logFile("Robot see " + ringRecArray.size() + " rings");
            if (ringRecArray.size()!=0) {
                // now let's translate from image pixel to robot coordinate
                Mat transformed = new Mat();
                ArrayList<Point> imgPosList = new ArrayList<Point>();
                for (Rect r : ringRecArray) {
                    Logger.logFile("Ring Image at " + (r.x + r.width / 2) + ", " + (r.y + r.height / 2));
                    imgPosList.add(new Point(r.x + r.width / 2, r.y + r.height / 2));
                }
                Core.perspectiveTransform(Converters.vector_Point2f_to_Mat(imgPosList), transformed, trans);
                ArrayList<Point> transPoints = new ArrayList<Point>();
                Converters.Mat_to_vector_Point2f(transformed, transPoints);
                for (Point p : transPoints) {
                    Log.d("Point", p.toString());
                    Logger.logFile("Ring robot coordinate at (cm) " + p.x + ", " + p.y);
                }

                // translate to field coordinate and merge to array
                for (Point p : transPoints) {
                    Vector2d worldCorr = getFieldCoordinate(robotHardware.getTrackingWheelLocalizer().getPoseEstimate(), p);
                    Logger.logFile("Ring field coordinate at (inch) " + worldCorr.getX() + ", " + worldCorr.getY());
                    int n = 0;
                    while (n < rings.size()) {
                        Vector2d r = rings.get(n);
                        //when two rings are less than 3 inches apart, assume it's the same ring
                        if (r.distTo(worldCorr) < 3) {
                            rings.remove(n);
                        }
                        else {
                            n++;
                        }
                    }
                    rings.add(worldCorr);
                }
            }
        }
        else {
            //DRIVE mode trigger Roadrunner movement
            drive.update();
            if (!drive.isBusy()) {
                if (arrayTraj.size()>0) {
                    Trajectory traj = arrayTraj.remove(0);
                    drive.followTrajectoryAsync(traj);
                }
                else {
                    doneDriving = true;
                }
            }
        }
    }

    @Override
    public void cleanUp() {

    }

    /**
     * picture mode done right away
     * drive mode complete when roadrunner is idle
     * @return
     */
    @Override
    public boolean isDone() {
        if(taskMode == TaskMode.DRIVE){
            return doneDriving;
        } else {
            return true;
        }
    }
}
