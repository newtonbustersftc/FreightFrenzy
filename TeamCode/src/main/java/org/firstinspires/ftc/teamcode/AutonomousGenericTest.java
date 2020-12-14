package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
        setUpTaskList();
        robotVision = robotHardware.getRobotVision();
        //robotVision.activateRecognition();
        robotHardware.getTrackingWheelLocalizer().setPoseEstimate(new Pose2d(-66, -33, 0));

        waitForStart();
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


        if (taskList.size()>0) {
            Logger.logFile("Task Prepare " + taskList.get(0));
            taskList.get(0).prepare();
        }
        // run until the end of the match (driver presses STOP)
        // run until the end of the match (driver presses STOP)
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

 */
        try {
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }
    }

    void setUpTaskList() {
        taskList = new ArrayList<RobotControl>();
//
//        Trajectory traj = robotHardware.mecanumDrive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(36, -15), 0)
//                .splineTo(new Vector2d(48, -15), 0)
//                .splineTo(new Vector2d(84, 6), 0)
//                .build();
//
//        Trajectory traj2 = robotHardware.mecanumDrive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(48, -15), Math.toRadians(180))
//                        .splineTo(new Vector2d(36, -15), Math.toRadians(180))
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build();
//
//        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, traj));
//        taskList.add(new RobotSleep(1000));
//        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, traj2));
        }
}
