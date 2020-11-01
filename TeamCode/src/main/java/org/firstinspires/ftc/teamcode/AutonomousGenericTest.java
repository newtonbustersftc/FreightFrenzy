package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;

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

        waitForStart();

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
        try {
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }
    }

    void setUpTaskList() {
        taskList = new ArrayList<RobotControl>();

        Trajectory traj = robotHardware.mecanumDrive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(36, -15), 0)
                .splineTo(new Vector2d(48, -15), 0)
                .splineTo(new Vector2d(84, 6), 0)
                .build();

        Trajectory traj2 = robotHardware.mecanumDrive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(48, -15), Math.toRadians(180))
                        .splineTo(new Vector2d(36, -15), Math.toRadians(180))
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build();

        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, traj));
        taskList.add(new RobotSleep(1000));
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, traj2));
    }
}
