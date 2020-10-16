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
    int skystonePosition = -1;
    RobotProfile.StartPosition startPosition;
    String startingPositionModes;
    String parkingLocation;

    SequentialComboTask pickUpTask;
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
        robotHardware.setClampPosition(RobotHardware.ClampPosition.INITIAL);
        driverOptions = new DriverOptions();
        Logger.logFile("Init completed");
        Logger.logFile("DistancePID:" + robotProfile.distancePID.p + ", " + robotProfile.distancePID.i + ", " + robotProfile.distancePID.d);
        Logger.logFile("DistancePID:" + robotProfile.headingPID.p + ", " + robotProfile.headingPID.i + ", " + robotProfile.headingPID.d);
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        try {
            String delaystring = prefs.getString(DELAY_PREF, "");
            delaystring = delaystring.replace(" sec", "");
            driverOptions.setDelay(Integer.parseInt(delaystring));
            Logger.logFile("delay: "+ this.delay);
            driverOptions.setParking(prefs.getString(PARKING_PREF,""));
            driverOptions.setStartingPositionModes(prefs.getString(START_POS_MODES_PREF, ""));
            driverOptions.setDeliverRoute(prefs.getString(DELIVER_ROUTE_PREF,""));
            driverOptions.setMoveFoundation(prefs.getString(FOUNDATION_PREF,""));
            driverOptions.setIsParkOnly(prefs.getString(PARKING_ONLY_PREF,""));
            Logger.logFile("parking: "+ driverOptions.getParking());
            Logger.logFile("startingPositionModes: "+ driverOptions.getStartingPositionModes());
            Logger.logFile("deliverRoute: " + driverOptions.getDeliverRoute());
            Logger.logFile("moveFoundation: " + driverOptions.getMoveFoundation());
            Logger.logFile("isParkOnly: " + driverOptions.getIsParkOnly());
            Logger.logFile("StoneOptions: " + driverOptions.getStoneOptions());
        } catch (Exception e) {
            this.delay = 0;
        }

        double heading;
         if (driverOptions.getStartingPositionModes().equals("RED_2")) {
            startPosition = RobotProfile.StartPosition.RED_2;
            heading = 0;
        }
        else if (driverOptions.getStartingPositionModes().equals("RED_3")) {
            startPosition = RobotProfile.StartPosition.RED_3;
            heading = 0;
        }
        else if (driverOptions.getStartingPositionModes().equals("BLUE_2")) {
            startPosition = RobotProfile.StartPosition.BLUE_2;
            heading =0;
        }
        else {
            startPosition = RobotProfile.StartPosition.BLUE_3;
            heading =0;
        }

        navigator.setInitPosition(0,0, 0);
        Logger.logFile("init x: " + robotProfile.robotStartPoints.get(startPosition).getX());
        Logger.logFile("init y: " + robotProfile.robotStartPoints.get(startPosition).getY());
        Logger.logFile("heading" + heading );

        SharedPreferences.Editor editor = prefs.edit();
        editor.putString(START_POS_MODES_PREF, driverOptions.getStartingPositionModes());
        editor.putString(PARKING_PREF, driverOptions.getParking());
        editor.apply();
    }

    @Override
    public void runOpMode() {
        initRobot();
        setUpTaskList();

        //
        //moveAndSlideToStone(new RobotPosition(0, 0, 0), new RobotPosition(70, 20, 0));
        taskList.add(new RobotSleep(10000));
        double origImu = robotHardware.getGyroAngle();

        waitForStart();


        robotHardware.setClampPosition(RobotHardware.ClampPosition.INITIAL);
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
