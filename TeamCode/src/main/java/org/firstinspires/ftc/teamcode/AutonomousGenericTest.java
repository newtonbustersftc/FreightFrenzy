package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AngleMath;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;


/**
 * 2019.10.26
 * Created by Ian Q.
 */
@TeleOp(name="AutonomousTest", group="Test")
public class AutonomousGenericTest extends LinearOpMode {

    RobotHardware robotHardware;
    RobotProfile robotProfile;
    RobotVision robotVision;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;
    Pose2d pos;
    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;

    public void initRobot() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        Logger.logFile("Init completed");
    }

    @Override
    public void runOpMode() {

        initRobot();
        robotHardware.initRobotVision();
        robotVision = robotHardware.getRobotVision();
        //robotHardware.getTrackingWheelLocalizer().setPoseEstimate(new Pose2d(-66, -33, 0));
        //robotHardware.getLocalizer().update();
        //robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("START"));
        long loopStart = System.currentTimeMillis();
        long loopCnt = 0;
        while (!isStarted()) {
            //RobotVision.AutonomousGoal goal = robotHardware.getRobotVision().getAutonomousRecognition(false);
            //telemetry.addData("goal",goal);
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt%100==0) {
                telemetry.addData("CurrPose", currPose);
                telemetry.addData("T265 CFD:",  ((RealSenseLocalizer)robotHardware.getLocalizer()).getT265Confidence());
                telemetry.addData("LoopTPS:", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.update();
            }
        }

        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        Logger.logFile("Recognition Result: " + robotVision.getAutonomousRecognition());
        taskList = new ArrayList<RobotControl>();
//        setupTaskList3();
        setupTaskList1();
        if (taskList.size()>0) {
            Logger.logFile("Task Prepare " + taskList.get(0));
            taskList.get(0).prepare();
        }
        // run until the end of the match (driver presses STOP)
        // run until the end of the match (driver presses STOP)
        long startTime = System.currentTimeMillis();
        int cnt = 100;
        double veloSum = 0;
        Logger.logFile("Main Task Loop started");

        while (opModeIsActive()) {
            loopCount++;
            robotHardware.getBulkData1();
            robotHardware.getBulkData2();
            robotHardware.getLocalizer().update();
            Logger.logFile("Pose:" + robotHardware.getLocalizer().getPoseEstimate());
            //Logger.logFile("Velocity:" + robotHardware.getLocalizer().getPoseVelocity());
            try {
                Logger.flushToFile();
            }
            catch (Exception ex) {
            }
            /*Specific test for motor velocity */
// append to shooting velocity csv file
            /* End Testing code */
            if (taskList.size() > 0) {
                taskList.get(0).execute();
                if (taskList.get(0).isDone()) {
                    Logger.logFile("Task Complete " + taskList.get(0));
                    taskList.get(0).cleanUp();
                    taskList.remove(0);
                    countTasks++;
                    telemetry.update();
                    if (taskList.size() > 0) {
                        taskList.get(0).prepare();
                    }
                }
            }
        }
        try {
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    void setupTaskList1() {
//        DriveConstraints constraints = new DriveConstraints(5.0, 5.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
//        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        velConstraint = getVelocityConstraint(35, 30, TRACK_WIDTH);
        accelConstraint = getAccelerationConstraint(30);
        // move to shoot
        Pose2d p0 = new Pose2d(0,0,0);
        Pose2d p1 = new Pose2d(-20, 0, 0);
        //Pose2d p2 = new Pose2d(10,5,0);
        Trajectory trj = robotHardware.mecanumDrive.trajectoryBuilder(p0, true)
                .splineTo(p1.vec(), p1.getHeading()+Math.PI)
                //.splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trj);
        taskList.add(moveTask1);
        taskList.add(new RobotSleep(500));
        Trajectory trj2 = robotHardware.mecanumDrive.trajectoryBuilder(new Pose2d(p1.getX(), p1.getY(), p1.getHeading()))
                .splineTo(p0.vec(), p0.getHeading())
                //.splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask2 = new SplineMoveTask(robotHardware.mecanumDrive, trj2);
        taskList.add(moveTask2);
        taskList.add(new RobotSleep(500));
        taskList.add(moveTask1);
        taskList.add(moveTask2);
    }

    void setupTaskList2() {
        PIDMecanumMoveTask pmm1 = new PIDMecanumMoveTask(robotHardware,robotProfile);
        pmm1.setPower(0.5);
        pmm1.setPath(new Pose2d(0,0,0), new Pose2d(20, 0, 0));

        taskList.add(pmm1);

        MecanumRotateMoveTask mrm1 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm1.setRotateHeading(new Pose2d(20,0,0), new Pose2d(20, 0, Math.PI/2));
        mrm1.setPower(0.3);
        taskList.add(mrm1);

        PIDMecanumMoveTask pmm2 = new PIDMecanumMoveTask(robotHardware, robotProfile);
        pmm2.setPath(new Pose2d(20,0,Math.PI/2), new Pose2d(20, -20, Math.PI/2));
        pmm2.setPower(0.3);
        taskList.add(pmm2);

        MecanumRotateMoveTask mrm2 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm2.setRotateHeading(new Pose2d(20,-20,Math.PI/2), new Pose2d(30, -20, 0));
        mrm2.setPower(0.3);
        taskList.add(mrm2);

//        PIDMecanumMoveTask pmm2 = new PIDMecanumMoveTask(robotHardware,robotProfile);
//        pmm2.setPower(0.3);
//        pmm2.setPath(new Pose2d(1,0,Math.PI/2), new Pose2d(10, -10, Math.PI/2));
//
//        taskList.add(pmm2);
    }

    void setupTaskList3() {
        MecanumRotateMoveTask mrm1 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm1.setRotateHeading(new Pose2d(00, 0, 0), new Pose2d(-15, -15, -Math.PI / 3));
        mrm1.setPower(0.3);
        taskList.add(mrm1);
        taskList.add(new RobotSleep(1000));
        MecanumRotateMoveTask mrm2 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm2.setRotateHeading(new Pose2d(-15, -15, -Math.PI / 3), new Pose2d(00, 0, 0));
        mrm2.setPower(0.3);
        taskList.add(mrm2);
    }
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
