package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;


@TeleOp(name="Newton DriverOpMode", group="Main")
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotNavigator navigator;
    RobotProfile robotProfile;


    Pose2d currPose;
    double fieldHeadingOffset;

    boolean fieldMode;
    boolean xAlreadyPressed = false;
    boolean yAlreadyPressed = false;
    boolean bAlreadyPressed = false;

    // DriveThru combos
    RobotControl currentTask = null;
    RobotVision robotVision;
    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        fieldMode = true; //robot starts in field orientation

        Logger.init();

        //robotHardware = RobotFactory.getRobotHardware(hardwareMap,robotProfile);
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotVision = robotHardware.getRobotVision();
        //robotVision.activateRecognition();
        robotVision.activateNavigationTarget();

        // Based on the Autonomous mode starting position, define the heading offset for field mode
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        setupCombos();

    }

    private void handleVision() {
        if(gamepad1.a){
            Pose2d currentPosition = robotVision.getNavigationLocalization();
            if(currentPosition!=null){
                robotHardware.getTrackingWheelLocalizer().setPoseEstimate(currentPosition);
            }
        }

//        telemetry.addData("Current Location", currentPosition);
//        List<Recognition> updatedRecognitions = robotVision.getRingRecognition();
//        if (updatedRecognitions != null) {
//            telemetry.addData("# Object Detected", updatedRecognitions.size());
//            // step through the list of recognitions and display boundary info.
//            int i = 0;
//            for (Recognition recognition : updatedRecognitions) {
//                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                        recognition.getLeft(), recognition.getTop());
//                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                        recognition.getRight(), recognition.getBottom());
//            }
//        }
    }

    @Override
    public void loop() {
        robotHardware.getBulkData1();
        robotHardware.getBulkData2();
        robotHardware.getTrackingWheelLocalizer().update();
        currPose = robotHardware.getTrackingWheelLocalizer().getPoseEstimate();

        // Robot movement
        handleMovement();
        handleVision();

        // toggle field mode on/off.
        // Driver 1: left trigger - enable; right trigger - disable
        if (gamepad1.left_trigger > 0) {
            fieldMode = true;
            fieldHeadingOffset = currPose.getHeading();
        } else if (gamepad1.right_trigger > 0) {
            fieldMode = false;  //good luck driving
        }


        if (!gamepad2.y) {
            yAlreadyPressed = false;
        }

        if (currentTask != null) {
            currentTask.execute();

            if (currentTask.isDone()) {
                currentTask.cleanUp();
                currentTask = null;
            }
        }

        if(gamepad1.b && !bAlreadyPressed){
            bAlreadyPressed = true;
            Pose2d shootingPose = new Pose2d(-5, -36, Math.toRadians(0));
            double shootingHeading = 0;
            Trajectory trajectory = robotHardware.getMecanumDrive().trajectoryBuilder(currPose, false).
                    splineToSplineHeading(shootingPose, 0).build();
            currentTask = new SplineMoveTask(robotHardware.getMecanumDrive(), trajectory);
            currentTask.prepare();
        } else if(!gamepad1.b && bAlreadyPressed) {
            bAlreadyPressed = false;
        }
        telemetry.addData("CurrPose", currPose);
        telemetry.addData("Field Mode", fieldMode);
    }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            Logger.logFile("DriverOpMode stop() called");
            robotVision.deactivateNavigationTarget();
            Logger.flushToFile();
        } catch (Exception e) {
        }
    }

    private void handleMovement() {
        if(currentTask instanceof SplineMoveTask){
            return;
        }
        double turn = gamepad1.right_stick_x/2;
        double power = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double moveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI/4.5;

        if (fieldMode) {
            moveAngle += currPose.getHeading() - fieldHeadingOffset - Math.PI/2;
        }

        if (gamepad1.left_bumper) {
            power = power/3.5;
            turn = turn/13;
        }
        /** what is this?!!
        if(gamepad1.x && !xAlreadyPressed){
            xAlreadyPressed = true;
            power = power/3.5;
            turn = turn/13;
        } else if(gamepad1.x && xAlreadyPressed){
            xAlreadyPressed = false;
            power = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            turn = gamepad1.right_stick_x;
        }
        */

        robotHardware.mecanumDriveTest(power, moveAngle, turn, 0);
    }

    void setupCombos() {
//        ArrayList<RobotControl> homePositionList = new ArrayList<RobotControl>();
//        homePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
//                robotProfile.hardwareSpec.liftPerStone + robotProfile.hardwareSpec.liftGrabExtra, 100));
//        homePositionList.add(new ClampStraightAngleTask(robotHardware, robotProfile));
//        homePositionList.add(new SetSliderPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.sliderOrigPos, 100));
//        homePositionList.add(new ClampOpenCloseTask(robotHardware, robotProfile, RobotHardware.ClampPosition.OPEN));
//        homePositionList.add(new SetLiftPositionTask(robotHardware, robotProfile, robotProfile.hardwareSpec.liftStoneBase +
//                robotProfile.hardwareSpec.liftHomeReadyPos, 100));
//        homePositionTask = new SequentialComboTask();
//        homePositionTask.setTaskList(homePositionList);
    }


}
