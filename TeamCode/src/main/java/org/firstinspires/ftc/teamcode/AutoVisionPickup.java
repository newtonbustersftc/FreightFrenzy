package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.ParallelComboTask;
import org.firstinspires.ftc.teamcode.RingHolderPosTask;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.RobotSleep;
import org.firstinspires.ftc.teamcode.RobotVision;
import org.firstinspires.ftc.teamcode.ShootOneRingTask;
import org.firstinspires.ftc.teamcode.ShooterMotorTask;
import org.firstinspires.ftc.teamcode.SplineMoveTask;
import org.firstinspires.ftc.teamcode.TaskReporter;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import org.apache.commons.math3.geometry.euclidean.twod.*;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;

@Autonomous(name="View Translation", group="Test")
public class AutoVisionPickup extends LinearOpMode {

    RobotHardware robotHardware;
    RobotProfile robotProfile;
    RobotVision robotVision;

    Mat trans;
    Mat invTrans;
    Pose2d shootingPose = new Pose2d(-5, -36, Math.toRadians(-10));
    static Pose2d startPose = new Pose2d(60/2.54, 0, Math.toRadians(180));
    static Line[] walls = new Line[4];
    static TrajectoryBuilder currBuilder;
    static DriveConstraints constraints = new DriveConstraints(20.0, 15.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
    static double lastHeading = 0;

    public static double FIELD_WIDTH = 144.0; // 12'
    public static double  ROBOT_WIDTH = 18.0;

    long loopCount = 0;

    static {
        walls[0] = new Line(new Vector2D(FIELD_WIDTH/2, FIELD_WIDTH/2/3), new Vector2D(FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);      // we don't have full
        walls[1] = new Line(new Vector2D(FIELD_WIDTH/2, FIELD_WIDTH/2), new Vector2D(-FIELD_WIDTH/2, FIELD_WIDTH/2), 0);
        walls[2] = new Line(new Vector2D(-FIELD_WIDTH/2, FIELD_WIDTH/2), new Vector2D(-FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);
        walls[3] = new Line(new Vector2D(-FIELD_WIDTH/2, -FIELD_WIDTH/2), new Vector2D(FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);
    }

    /**
     * This function will take in a set of 4 coordinates, and the corresponding camera image coordinate,
     * use OpenCV PerspectiveTransform to map image coordinate into field coordinate
     *
     ******************************
     *             0              *
     *                            *
     *                            *
     *  1                      2  *
     *                            *
     *             3              *
     *                            *
     *                            *
     *             R              *
     ******************************
     */
    void generateProfile() {
        ArrayList<Point> imgList = new ArrayList<Point>();
        ArrayList<Point> fieldList = new ArrayList<Point>();

        imgList.add(new Point(284, 141));
        imgList.add(new Point(50, 209));
        imgList.add(new Point(561, 203));
        imgList.add(new Point(267, 361));

        fieldList.add(new Point(180 + ROBOT_WIDTH/2 * 2.54, 0));
        fieldList.add(new Point(120 + ROBOT_WIDTH/2 * 2.54, 40));
        fieldList.add(new Point(120 + ROBOT_WIDTH/2 * 2.54, -50));
        fieldList.add(new Point(60 + ROBOT_WIDTH/2 * 2.54, 0));

        Mat imgMat = Converters.vector_Point2f_to_Mat(imgList);
        Mat fieldMat = Converters.vector_Point2f_to_Mat(fieldList);
        trans = Imgproc.getPerspectiveTransform(imgMat, fieldMat);
        invTrans = Imgproc.getPerspectiveTransform(fieldMat, imgMat);
    }

    // Convert from cm to inch, also rotate and shift based on current Robot Pose
    Vector2d getFieldCoordinate(Pose2d currPose, Point robotViewPt) {
        double fieldX = currPose.getX() + (robotViewPt.x * Math.cos(currPose.getHeading()) - robotViewPt.y*Math.sin(currPose.getHeading()))/2.54;
        double fieldY = currPose.getY() + (robotViewPt.x * Math.sin(currPose.getHeading()) + robotViewPt.y * Math.cos(currPose.getHeading()))/2.54;
        return new Vector2d(fieldX, fieldY);
    }

    /**
     * We are at starting p1, need to add the move to p2 with p3 as next point
     */
    void addMove(Pose2d p1, Vector2d p2, Vector2d p3) {
        Logger.logFile("Add Move " + p1 + " -> " + p2 + " -> " + p3);
        Line p1ToP2 = new Line(new Vector2D(p1.getX(), p1.getY()), new Vector2D(p2.getX(), p2.getY()), 0);
        Line p2ToP3 = new Line(new Vector2D(p2.getX(), p2.getY()), new Vector2D(p3.getX(), p3.getY()), 0 );
        boolean reverse = Math.abs((p1.getHeading() - p1ToP2.getAngle()))>Math.PI/2;  // when the angle of more than 90 degree, reverse
        // The angle should try to pass P2 to P3
        if (Math.abs(p1ToP2.getAngle() - p2ToP3.getAngle())>Math.PI) {
            lastHeading = (p2ToP3.getAngle() - p1ToP2.getAngle())/2 + Math.signum(p1ToP2.getAngle() - p2ToP3.getAngle()) * Math.PI;
        }
        else {
            lastHeading = (p1ToP2.getAngle() + p2ToP3.getAngle()) / 2;
        }
        if (currBuilder==null) {
            currBuilder = new TrajectoryBuilder(p1, reverse, constraints);
        }
        currBuilder.splineToSplineHeading(new Pose2d(p2.getX(), p2.getY(), lastHeading), lastHeading);
    }

    /**
     * Define a line segment from the current robot position to shooting position, select
     * the rings that are close to this line segment.  If ring is too close to the wall, add
     * distance adjustment to that ring before ranking
     * @return
     */
    public ArrayList<Vector2d> selectRings(ArrayList<Vector2d> rings) {
        Line segLine = new Line(new Vector2D(startPose.getX(), startPose.getY()), new Vector2D(shootingPose.getX(), shootingPose.getY()), 0);
        Segment seg = new Segment(new Vector2D(startPose.getX(), startPose.getY()), new Vector2D(shootingPose.getX(), shootingPose.getY()), segLine);
        // sort the rings by their distance to the segment
        DistToSeg comp = new DistToSeg(walls, seg);

        ArrayList<Vector2d> tmp = new ArrayList<Vector2d>(rings);
        tmp.sort(comp);
        ArrayList<Vector2d> resp = new ArrayList<Vector2d>();
        for(int i=0; i<4; i++) {
            resp.add(tmp.get(i));
        }
        return resp;
    }

    ArrayList<Trajectory> pickUpAndShoot(Vector2d r1, Vector2d r2, Vector2d r3) {
        Logger.logFile("pickUpAndShoot " + r1 + "," + r2 + ", " + r3);
        ArrayList<Trajectory> moves = new ArrayList<Trajectory>();
        currBuilder = null;
        addMove(startPose, r1, r2);
        addMove(new Pose2d(r1.getX(), r1.getY(), lastHeading), r2, r3);
        addMove(new Pose2d(r2.getX(), r2.getY(), lastHeading), r3, shootingPose.vec());
        currBuilder.splineToSplineHeading(shootingPose, shootingPose.getHeading());
        moves.add(currBuilder.build());
        return moves;
    }

    // Enumerate all chosen ring possible pick up order and get quickest one
    ArrayList<Trajectory> generatePath(ArrayList<Vector2d> rings) {
        ArrayList<Vector2d> chosen = selectRings(rings);
        Logger.logFile("Selected rings - " + chosen.size());
        // build a path for different pick up orders
        ArrayList<Trajectory> bestMove = new ArrayList<Trajectory>();
        int moveExamined = 0;
        long startTime = System.currentTimeMillis();
        double bestDuration = 10000;
        for (int a = 0; a < chosen.size(); a++) {
            for (int b = 0; b < chosen.size(); b++) {
                if (a != b) {
                    for (int c = 0; c < chosen.size(); c++) {
                        if (a != b && a != c && b != c) {
                            double currDuration = 0;
                            moveExamined++;
                            ArrayList<Trajectory> move = pickUpAndShoot(chosen.get(a), chosen.get(b), chosen.get(c));
                            for(Trajectory t : move) {
                                currDuration = currDuration + t.duration();
                            }
                            if (bestDuration > currDuration) {
                                bestDuration = currDuration;
                                bestMove = move;
                            }
                        }
                    }
                }
            }
        }
        long timeSpent = System.currentTimeMillis() - startTime;
        Logger.logFile("Example " + moveExamined + " time: " + timeSpent + " avg: " + 1.0*timeSpent/moveExamined);
        return bestMove;
    }

    public void initRobot() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotHardware.initRobotVision();
        robotVision = robotHardware.getRobotVision();
        robotHardware.getTrackingWheelLocalizer().setPoseEstimate(startPose);
        Logger.logFile("Init completed");
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    @Override
    public void runOpMode() {
        generateProfile();
        initRobot();
        waitForStart();
        // take a picture of the field to get rings
        ArrayList<Rect> ringRecArray = robotVision.getRings(true);
        Logger.logFile("Robot see " + ringRecArray.size() + " rings");

        // now let's translate from image pixel to field
        Mat transformed = new Mat();
        ArrayList<Point> imgPosList = new ArrayList<Point>();
        for(Rect r : ringRecArray) {
            Logger.logFile("Ring Image at " + (r.x + r.width/2) + ", " + (r.y + r.height/2));
            imgPosList.add(new Point(r.x + r.width/2, r.y+r.height/2));
        }
        Core.perspectiveTransform(Converters.vector_Point2f_to_Mat(imgPosList), transformed, trans);
        ArrayList<Point> transPoints = new ArrayList<Point>();
        Converters.Mat_to_vector_Point2f(transformed, transPoints);
        for(Point p:transPoints) {
            Log.d("Point", p.toString());
            Logger.logFile("Ring robot coordinate at (cm) " + p.x + ", " + p.y);
        }

        // translate to field coordinate and add to array
        ArrayList<Vector2d> ringVec2dArray = new ArrayList<Vector2d>();
        for(Point p:transPoints) {
            Vector2d worldCorr = getFieldCoordinate(startPose, p);
            Logger.logFile("Ring field coordinate at (inch) " + worldCorr.getX() + ", " + worldCorr.getY());
            ringVec2dArray.add(worldCorr);
        }
        ArrayList<Trajectory> trjArray = generatePath(ringVec2dArray);
        Logger.logFile("Got Path " + trjArray.size());

        ArrayList<RobotControl> taskList = new ArrayList<RobotControl>();

        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints extraSlowConstraints = new DriveConstraints(5.0, 5.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);

        // move to shoot
        Pose2d p0 = startPose;
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjArray.get(0));

        taskList.add(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.NORMAL));
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, true));
        taskList.add(moveTask1);
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.STOP));
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));

        TaskReporter.report(taskList);
        Logger.logFile("Task list items: " + taskList.size());
        Logger.flushToFile();

        if (taskList.size() > 0) {
            taskList.get(0).prepare();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            loopCount++;

            robotHardware.getBulkData1();
            robotHardware.getBulkData2();

            if (taskList.size() > 0) {
                taskList.get(0).execute();

                if (taskList.get(0).isDone()) {
                    Logger.logFile("MainTaskComplete: " + taskList.get(0) + " Pose:" + robotHardware.getTrackingWheelLocalizer().getPoseEstimate());
                    Logger.flushToFile();

                    taskList.get(0).cleanUp();
                    taskList.remove(0);

                    telemetry.update();

                    if (taskList.size() > 0) {
                        taskList.get(0).prepare();
                    }
                }
            }
        }
        // Regardless, open the clamp to save the servo
        try {
            Logger.logFile("Autonomous - Final Location:");
            Logger.flushToFile();
        } catch (Exception ex) {
        }

        robotHardware.setMotorStopBrake(false);
    }

    class DistToSeg implements Comparator<Vector2d> {
        Segment seg;
        Line[] walls;

        public DistToSeg(Line[] walls, Segment seg) {
            this.seg = seg;
            this.walls = walls;
        }

        @Override
        /**
         * Sort by the distance of ring to the line segment.  If ring is too close to wall, distance increases by Robot Width
         */
        public int compare(Vector2d a, Vector2d b) {
            Vector2D a2D = new Vector2D(a.getX(), a.getY());
            Vector2D b2D = new Vector2D(b.getX(), b.getY());

            double aX = 0, bX = 0;
            for (int i = 0; i < walls.length; i++) {
                if (walls[i].distance(a2D) < ROBOT_WIDTH / 2) {     // if too close to the wall, add adjustment
                    aX = ROBOT_WIDTH;
                }
                if (walls[i].distance(b2D) < ROBOT_WIDTH / 2) {     // if too close to the wall, add adjustment
                    bX = ROBOT_WIDTH;
                }
            }
            return (int) Math.signum((seg.distance(a2D) + aX) - (seg.distance(b2D) + bX));
        }
    }

    class CompareDuration implements Comparator<ArrayList<Trajectory>> {
        @Override
        public int compare(ArrayList<Trajectory> o1, ArrayList<Trajectory> o2) {
            double t1 = 0;
            double t2 = 0;
            for(Trajectory t : o1) {
                t1 = t1 + t.duration();
            }
            for(Trajectory t : o2) {
                t2 = t2 + t.duration();
            }

            return (int)Math.signum(t1 - t2);
        }
    }
}
