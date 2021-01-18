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
        setUpTaskList();
        //List<Recognition> updatedRecognitions = robotVision.getRingRecognition();


/*

        if (updatedRecognitions != null) {
            Logger.logFile("# Object Detected: " + updatedRecognitions.size());

            // step through the list of recognitions and display boundary info.
            int i = 0;
            pos = new Pose2d(0, -48, -Math.PI/2);
            for (Recognition recognition : updatedRecognitions) {
                Logger.logFile(String.format("label (%d) ", i) + ", " + recognition.getLabel());
                Logger.logFile("  left,top (%d) " + i +" %.03f , %.03f "+
                        recognition.getLeft() + ", " + recognition.getTop());
                Logger.logFile("  right,bottom (%d) "+ i+ " %.03f , %.03f "+
                        recognition.getRight() + ", " + recognition.getBottom());


                if(recognition.getLabel() == "Quad"){
                    Logger.logFile("got quad");
                    pos = new Pose2d(49, -48, -Math.PI/2);
                } else if(recognition.getLabel() == "Single") {
                    Logger.logFile(("got single"));
                    pos = new Pose2d(25, -33, 0);
                }


            }
            Trajectory moveRobot = robotHardware.mecanumDrive.trajectoryBuilder(new Pose2d(-66, -33, 0))
                    .splineTo(new Vector2d(-21, -9), 0)
                    .splineTo(pos.vec(), pos.getHeading())
                    .build();

            taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, moveRobot));
        }
        else {
            Logger.logFile("getRingRecognition returns null");
        }
*/

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
            if ((System.currentTimeMillis() - startTime > 4000) && (cnt>0)) {
                veloSum += robotHardware.getEncoderVelocity(RobotHardware.EncoderType.SHOOTER);
                cnt--;
                if (cnt==0) {
                    Log.d("Shooter Velocity", "" + veloSum/100);
                    Logger.logFile("Shooter Velocity " + veloSum/100);
                }
            }
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

    void setUpTaskList() {
        taskList = new ArrayList<RobotControl>();
        taskList.add(new ShooterMotorTask(robotHardware, robotProfile, true, robotProfile.hardwareSpec.shootVelocity));
        taskList.add(new RobotSleep(5000));
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
        par1.addTask(new MoveArmTask(robotHardware, robotProfile, RobotHardware.ArmPosition.DELIVER, 1000));
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
}
