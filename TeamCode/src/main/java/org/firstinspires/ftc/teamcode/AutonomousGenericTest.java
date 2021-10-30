package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        robotVision.getAutonomousRecognition();
        taskList = new ArrayList<RobotControl>();
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

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }
}
