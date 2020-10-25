package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELAY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.DELIVER_ROUTE_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.FOUNDATION_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_ONLY_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.PARKING_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;
import static org.firstinspires.ftc.teamcode.AutonomousOptions.STONE_PREF;

/**
 * 2019.10.26
 * Created by Ian Q.
 */
//@TeleOp(name="AutonomousTest", group="Test")
public class AutonomousGenericTest extends LinearOpMode {

    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;
    DriverOptions driverOptions;

    int leftEncoderCounts;
    int rightEncoderCounts;
    int horizontalEncoderCounts;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;

    private int delay;

    public void initRobot() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        navigator = new RobotNavigator(robotProfile);
        navigator.reset();
        navigator.setInitPosition(0, 0, 0);
        driverOptions = new DriverOptions();
        Logger.logFile("Init completed");
        Logger.logFile("DistancePID:" + robotProfile.distancePID.p + ", " + robotProfile.distancePID.i + ", " + robotProfile.distancePID.d);
        Logger.logFile("DistancePID:" + robotProfile.headingPID.p + ", " + robotProfile.headingPID.i + ", " + robotProfile.headingPID.d);
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        try {

        } catch (Exception e) {
            this.delay = 0;
        }

        double heading;

        navigator.setInitPosition(0,0, 0);

    }

    @Override
    public void runOpMode() {
        initRobot();
        setUpTaskList();

        taskList.add(new RobotSleep(10000));
        double origImu = robotHardware.getGyroAngle();

        waitForStart();

        if (taskList.size()>0) {
            taskList.get(0).prepare();
        }
        // run until the end of the match (driver presses STOP)
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            loopCount++;
            robotHardware.getBulkData1();
            robotHardware.getBulkData2();
            leftEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT);
            rightEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT);
            horizontalEncoderCounts = robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL);
            navigator.updateEncoderPos(leftEncoderCounts, rightEncoderCounts, horizontalEncoderCounts);

            if (taskList.size() > 0) {
                taskList.get(0).execute();
                if (taskList.get(0).isDone()) {
                    double newImu = robotHardware.getGyroAngle();
                    Logger.logFile("IME Rotate:" + (newImu - origImu));
                    Logger.logFile("TaskComplete: " + taskList.get(0) + " Position:" + navigator.getWorldX() + "," + navigator.getWorldY() + " :" + navigator.getHeading());
                    Logger.flushToFile();
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
        navigator.reset();
        taskList = new ArrayList<RobotControl>();
    }
}
