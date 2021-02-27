package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.io.File;
import java.util.ArrayList;
import java.util.List;


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

        robotVision = robotHardware.getRobotVision();
        //robotVision.activateRecognition();
        //robotHardware.getTrackingWheelLocalizer().setPoseEstimate(new Pose2d(-66, -33, 0));
        robotHardware.getTrackingWheelLocalizer().update();
        robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("START"));

        waitForStart();
        setUpTaskList3();

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

    void setUpTaskList2() {
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints constraints = new DriveConstraints(10.0, 5.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        SequentialComboTask powerBar = new SequentialComboTask();
        robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("SHOOT-START"));
        Pose2d p0 = getProfilePose("SHOOT-START");
        Pose2d p1 = getProfilePose("SHOOT-POWER-BAR-1");
        Trajectory traj1 = robotHardware.getMecanumDrive().trajectoryBuilder(p0, moveFast)
                .splineToSplineHeading(p1, p1.getHeading())
                .build();
        powerBar.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        powerBar.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));
        powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj1));
        powerBar.addTask(new ShootOneRingTask(robotHardware, robotProfile));
        powerBar.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));

        Pose2d p2 = getProfilePose("SHOOT-POWER-BAR-2");
        Trajectory traj2 = robotHardware.getMecanumDrive().trajectoryBuilder(p1, constraints)
                .lineToLinearHeading(p2, constraints)
                .build();
        powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj2));
        powerBar.addTask(new ShootOneRingTask(robotHardware, robotProfile));
        powerBar.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));

        Pose2d p3 = getProfilePose("SHOOT-POWER-BAR-3");
        Trajectory traj3 = robotHardware.getMecanumDrive().trajectoryBuilder(p2, constraints)
                .lineToLinearHeading(p3, constraints)
                .build();
        powerBar.addTask(new SplineMoveTask(robotHardware.getMecanumDrive(), traj3));
        powerBar.addTask(new ShootOneRingTask(robotHardware, robotProfile));
        powerBar.addTask(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList = new ArrayList<RobotControl>();
        taskList.add(powerBar);
    }

    void setUpTaskList() {
        taskList = new ArrayList<RobotControl>();
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));

        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, true, robotProfile.hardwareSpec.shootVelocity));
        taskList.add(new RobotSleep(3000));
        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    void setUpTaskList1() {
        taskList = new ArrayList<RobotControl>();

        DriveConstraints constraints = new DriveConstraints(20.0, 10.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        // move to shoot
        Pose2d p0 = getProfilePose("START");
        Pose2d p1 = getProfilePose("TRANSIT");
        Pose2d p2 = getProfilePose("SHOOT");
        Trajectory trjShoot = robotHardware.mecanumDrive.trajectoryBuilder(p0, moveFast)
                .splineTo(p1.vec(), p1.getHeading())
                .splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trjShoot);
        ParallelComboTask par1 = new ParallelComboTask();
        par1.addTask(moveTask1);
        SequentialComboTask seq1 = new SequentialComboTask();
        seq1.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.HOLD, 1000));
        seq1.addTask(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.UP));
        par1.addTask(seq1);
        par1.addTask(new ShooterMotorTask(robotHardware, robotProfile, true));
        taskList.add(par1);
        // Shooting action
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShootOneRingTask(robotHardware, robotProfile));
        taskList.add(new RobotSleep(robotProfile.hardwareSpec.shootDelay));
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, false));
    }

    void setUpTaskList3() {
        taskList = new ArrayList<RobotControl>();
        taskList.add(new RingHolderPosTask(robotHardware, robotProfile, RingHolderPosTask.RingHolderPosition.DOWN));
        taskList.add(new RingPusherPosTask(robotHardware, robotProfile, RobotHardware.RingPusherPosition.DOWN));
        taskList.add(new IntakeMotorTask(robotHardware, robotProfile, IntakeMotorTask.IntakeMode.NORMAL));
        Pose2d p0 = getProfilePose("START");
        Pose2d p1 = new Pose2d(p0.getX()+30, p0.getY(), p0.getHeading());
        Trajectory trjMov = robotHardware.mecanumDrive.trajectoryBuilder(p0)
                .splineTo(p1.vec(), p1.getHeading()).build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, trjMov));
        taskList.add(new RingPusherPosTask(robotHardware, robotProfile, RobotHardware.RingPusherPosition.UP));
    }
}
