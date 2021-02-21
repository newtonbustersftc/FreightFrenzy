package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.BulkMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RingPickupPathGenerator;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;

import java.util.ArrayList;

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

    static ArrayList<Vector2d> rings = null;
    static Mat trans = null;
    ArrayList<Trajectory> arrayTraj;
    boolean doneDriving;

    public static double  ROBOT_WIDTH = 18.0;


    public AutoDriveShootTask(RobotHardware robotHardware, RobotProfile robotProfile, TaskMode taskMode){
        this.robotHardware = robotHardware;
        this.taskMode = taskMode;
        this.robotProfile = robotProfile;
        this.drive = robotHardware.getMecanumDrive();
    }

    public String toString() {
        return "AutoDriveShoot " + taskMode;
    }

    void init() {
        ArrayList<Point> imgList = new ArrayList<Point>();
        ArrayList<Point> fieldList = new ArrayList<Point>();

        imgList.add(new Point(282, 139));
        imgList.add(new Point(43, 203));
        imgList.add(new Point(560, 203));
        imgList.add(new Point(264, 357));

        fieldList.add(new Point(180 + ROBOT_WIDTH/2 * 2.54, 0));
        fieldList.add(new Point(120 + ROBOT_WIDTH/2 * 2.54, 40));
        fieldList.add(new Point(120 + ROBOT_WIDTH/2 * 2.54, -50));
        fieldList.add(new Point(60 + ROBOT_WIDTH/2 * 2.54, 0));

        Mat imgMat = Converters.vector_Point2f_to_Mat(imgList);
        Mat fieldMat = Converters.vector_Point2f_to_Mat(fieldList);
        trans = Imgproc.getPerspectiveTransform(imgMat, fieldMat);

        rings = new ArrayList<Vector2d>();
    }

    Vector2d getFieldCoordinate(Pose2d currPose, Point robotViewPt) {
        double fieldX = currPose.getX() + (robotViewPt.x * Math.cos(currPose.getHeading()) - robotViewPt.y*Math.sin(currPose.getHeading()))/2.54;
        double fieldY = currPose.getY() + (robotViewPt.x * Math.sin(currPose.getHeading()) + robotViewPt.y * Math.cos(currPose.getHeading()))/2.54;
        return new Vector2d(fieldX, fieldY);
    }



    @Override
    public void prepare() {
        if(trans == null){
            init();
        }
        if(taskMode == TaskMode.FIRST_PIC){
            rings.clear();
        } else if(taskMode == TaskMode.DRIVE){
            RingPickupPathGenerator ringPickupPathGenerator = new RingPickupPathGenerator(robotHardware.getTrackingWheelLocalizer().getPoseEstimate(),
                    robotProfile.getProfilePose("SHOOT-DRIVER"));
            long start = System.currentTimeMillis();
            arrayTraj = ringPickupPathGenerator.generatePath(rings);
            Logger.logFile("Path Gen Time: " + (System.currentTimeMillis() - start));
            Trajectory traj = arrayTraj.remove(0);
            drive.followTrajectoryAsync(traj);
            doneDriving = false;
        }
    }

    @Override
    public void execute() {
        if(taskMode == TaskMode.MORE_PIC || taskMode == TaskMode.FIRST_PIC) {
            ArrayList<Rect> ringRecArray = robotHardware.getRobotVision().getRings();
            Logger.logFile("Robot see " + ringRecArray.size() + " rings");
            if (ringRecArray.size()!=0) {
                // now let's translate from image pixel to field
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

                // translate to field coordinate and add to array
                for (Point p : transPoints) {
                    Vector2d worldCorr = getFieldCoordinate(robotHardware.getTrackingWheelLocalizer().getPoseEstimate(), p);
                    Logger.logFile("Ring field coordinate at (inch) " + worldCorr.getX() + ", " + worldCorr.getY());
                    int n = 0;
                    while (n < rings.size()) {
                        Vector2d r = rings.get(n);
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

    @Override
    public boolean isDone() {
        if(taskMode == TaskMode.DRIVE){
            return doneDriving;
        } else {
            return true;
        }
    }
}
