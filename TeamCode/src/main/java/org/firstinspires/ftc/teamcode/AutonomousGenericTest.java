package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AngleMath;

import java.io.File;
import java.util.ArrayList;


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
        waitForStart();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        Logger.logFile("Recognition Result: " + robotVision.getAutonomousRecognition());
        taskList = new ArrayList<RobotControl>();
        setupTaskList2();
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
        DriveConstraints constraints = new DriveConstraints(5.0, 5.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        // move to shoot
        Pose2d p0 = new Pose2d(0,0,0);
        Pose2d p1 = new Pose2d(0, 5, 0);
        //Pose2d p2 = new Pose2d(10,5,0);
        Trajectory trj = robotHardware.mecanumDrive.trajectoryBuilder(p0)
                .forward(10, constraints)
                //.splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trj);
        MecanumRotateTask rot = new MecanumRotateTask(robotHardware.mecanumDrive, Math.PI/2);
        taskList.add(moveTask1);
        taskList.add(rot);
    }

    void setupTaskList2() {
        PIDMecanumMoveTask pmm1 = new PIDMecanumMoveTask(robotHardware,robotProfile);
        pmm1.setPower(0.5);
        pmm1.setPath(new Pose2d(0,0,0), new Pose2d(40, 0, 0));

        taskList.add(pmm1);

        MecanumRotateMoveTask mrm1 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm1.setRotateHeading(new Pose2d(40,0,0), new Pose2d(40, 0, Math.PI/2), AngleMath.Direction.ANTI_CLOCKWISE);
        mrm1.setPower(0.3);
        taskList.add(mrm1);

        PIDMecanumMoveTask pmm2 = new PIDMecanumMoveTask(robotHardware, robotProfile);
        pmm2.setPath(new Pose2d(40,0,Math.PI/2), new Pose2d(40, -40, Math.PI/2));
        pmm2.setPower(0.3);
        taskList.add(pmm2);

        MecanumRotateMoveTask mrm2 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        mrm2.setRotateHeading(new Pose2d(40,-40,Math.PI/2), new Pose2d(45, -40, 0), AngleMath.Direction.CLOCKWISE);
        mrm2.setPower(0.3);
        taskList.add(mrm2);

//        PIDMecanumMoveTask pmm2 = new PIDMecanumMoveTask(robotHardware,robotProfile);
//        pmm2.setPower(0.3);
//        pmm2.setPath(new Pose2d(1,0,Math.PI/2), new Pose2d(10, -10, Math.PI/2));
//
//        taskList.add(pmm2);
    }
}
