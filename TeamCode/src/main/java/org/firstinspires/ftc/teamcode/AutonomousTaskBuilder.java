package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.lifecycle.Lifecycle;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.arcrobotics.ftclib.geometry.Vector2d;
//import org.firstinspires.ftc.teamcode.
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AngleMath;

import java.util.ArrayList;

/**
 * sophia 11-6-21 14:32
 * marianna 11-6-21 09:00-14:30
 * Reuse keystone pid mecanum movements for our first meet.
 */

public class AutonomousTaskBuilder {
    RobotProfile robotProfile;
    RobotHardware robotHardware;
    ArrayList<RobotControl> taskList = new ArrayList<RobotControl>();
    DriverOptions driverOptions;
    SampleMecanumDrive drive;
    Pose2d startPos;
    int delay;
    String startingPositionModes;
    String parking;
    String deliverRoute;
    RobotVision.AutonomousGoal goal;
    Pose2d preParkPos;
    Pose2d parkPos;
    AngleMath.Direction direction;

    RobotHardware.LiftPosition targetLiftLevel;

    public AutonomousTaskBuilder(DriverOptions driverOptions, RobotHardware robotHardware, RobotProfile robotProfile, RobotVision.AutonomousGoal goal) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;

        this.driverOptions = driverOptions;
        this.delay = driverOptions.getDelay();
        this.startingPositionModes = driverOptions.getStartingPositionModes();
        this.parking = driverOptions.getParking();
        this.deliverRoute = driverOptions.getDeliveryRoutes();
        this.goal = goal;
        this.robotProfile = robotProfile;

        switch (goal) {
            case MIDDLE:
                this.targetLiftLevel = RobotHardware.LiftPosition.MIDDLE;
                break;
            case LEFT:
                this.targetLiftLevel = RobotHardware.LiftPosition.BOTTOM;
                break;
            case RIGHT:
            default:
                this.targetLiftLevel = RobotHardware.LiftPosition.TOP;
                break;
        }
    }

    public ArrayList<RobotControl> buildTaskList(RobotVision.AutonomousGoal goal) {
        if (delay>0) {
            taskList.add(new RobotSleep(delay*1000));
        }
//        buildParkOnlyTasks();
        if (startingPositionModes.endsWith("DUCK")) {
             buildDuckTasks(startingPositionModes);
        }
        else {
             buildDepotTasks(startingPositionModes);
        }
        robotHardware.getLocalizer().setPoseEstimate(startPos);
        return taskList;
    }

//    public void buildParkOnlyTasks(){
//        startPos = robotProfile.getProfilePose("BLUE_DUCK_START");
//        Pose2d parkPos = robotProfile.getProfilePose("BLUE_DUCK_PARKWALL");
//        Trajectory traj1 = drive.trajectoryBuilder(startPos)
//                .splineTo(new Vector2d(parkPos.getX(), parkPos.getY()), parkPos.getHeading())
//                .build();
//        taskList.add(new SplineMoveTask(drive, traj1));
//    }

    public void buildDepotTasks(String startPosStr){
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(15, 15, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(100, 100, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((15));
        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d preHubPos = robotProfile.getProfilePose(startPosStr + "_PREHUB");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_HUB");
        Pose2d warehousePickupPos = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL");
        Pose2d preParkPos, parkPos;
        Pose2d prePickPos;
        if (parking.endsWith("WALL")) {
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREWALL");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKWALL");
        }
        else if(parking.endsWith("CENTRAL")){
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PRECENTRAL");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKCENTRAL");
        }
        else { //if(parking.endsWith("STORAGE")) {
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREPARKSTORAGE");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKSTORAGE");
        }
        prePickPos = robotProfile.getProfilePose(startPosStr + "_PREWALL");

        // move out of the wall
        Trajectory traj0 =  drive.trajectoryBuilder(startPos)
                .strafeTo(preHubPos.vec())
                .build();
        taskList.add(new SplineMoveTask(drive, traj0));

        // move to hub
        ParallelComboTask par1 = new ParallelComboTask();
        Trajectory traj1 = drive.trajectoryBuilder(preHubPos, true)
                .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI, fastVelConstraints, accConstraint)
                .build();
        par1.addTask(new SplineMoveTask(drive, traj1));
        par1.addTask(new LiftBucketTask(robotHardware, robotProfile, targetLiftLevel));
        taskList.add(par1);
        // deliver to hub
        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        //*
        //Between delivering to hub and parking:
        //Pick up block, drive to hub and deliver.
//        Trajectory traj2 = drive.trajectoryBuilder(hubPos)
//                .splineTo(prePickPos.vec(), prePickPos.getHeading(),fastVelConstraints, accConstraint)
//                .splineTo(warehousePickupPos.vec(), warehousePickupPos.getHeading(), fastVelConstraints, accConstraint)
//                .build();
//        ParallelComboTask par2 = new ParallelComboTask();
//        par2.addTask(new SplineMoveTask(drive, traj2));
//        par2.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
//        par2.addTask(new IntakeTask(robotHardware, robotProfile));
//        taskList.add(par2);
//
//        Trajectory traj3 = drive.trajectoryBuilder(warehousePickupPos, true)
//                .splineTo(prePickPos.vec(), prePickPos.getHeading()+Math.PI, fastVelConstraints, accConstraint)
//                .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI)
//                .build();
//        ParallelComboTask par3 = new ParallelComboTask();
//        par3.addTask(new SplineMoveTask(drive, traj3));
//        par3.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.TOP));
//        taskList.add(par3);
//
//        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));
//
//        Trajectory traj4 = drive.trajectoryBuilder(hubPos)
//                .splineTo(prePickPos.vec(), prePickPos.getHeading(), fastVelConstraints, accConstraint)
//                .splineTo(warehousePickupPos.vec(), warehousePickupPos.getHeading(), fastVelConstraints, accConstraint)
//                .build();
//        ParallelComboTask par4 = new ParallelComboTask();
//        par4.addTask(new SplineMoveTask(drive, traj4));
//        par4.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
//        par4.addTask(new IntakeTask(robotHardware, robotProfile));
//        taskList.add(par4);
//
//        Trajectory traj5 = drive.trajectoryBuilder(warehousePickupPos, true)
//                .splineTo(prePickPos.vec(), prePickPos.getHeading()+Math.PI, fastVelConstraints, accConstraint)
//                .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI)
//                .build();
//
//        ParallelComboTask par5 = new ParallelComboTask();
//        par5.addTask(new SplineMoveTask(drive, traj5));
//        par5.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.TOP));
//        taskList.add(par5);
//
//        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

//        Pose2d preCentralPos = robotProfile.getProfilePose(startPosStr + "_PRECENTRAL");
//        Pose2d parkCentralPos = robotProfile.getProfilePose(startPosStr + "_PARKCENTRAL");
        Trajectory traj6 = drive.trajectoryBuilder(hubPos)
//                .splineTo(new Vector2d(preCentralPos.getX(), preCentralPos.getY()), preCentralPos.getHeading(), fastVelConstraints, accConstraint)
//                .splineTo(new Vector2d(parkCentralPos.getX(), parkCentralPos.getY()), parkCentralPos.getHeading(), fastVelConstraints, accConstraint)
                .splineTo(preParkPos.vec(), preParkPos.getHeading(), fastVelConstraints, accConstraint)
                .splineTo(parkPos.vec(), parkPos.getHeading(), fastVelConstraints, accConstraint)
                .build();
        ParallelComboTask par6 = new ParallelComboTask();
        par6.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
        par6.addTask(new SplineMoveTask(drive, traj6));
        taskList.add(par6);
    }

    public void buildDuckTasks(String startPosStr){
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(15, 15, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((15));
        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d preHubPos = robotProfile.getProfilePose(startPosStr + "_PREHUB");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_HUB");
        Pose2d duckSpinPos = robotProfile.getProfilePose(startPosStr + "_CAROUSEL");
        Pose2d afterSpinPos = robotProfile.getProfilePose(startPosStr + "_AFTERCAROUSEL");
        if (parking.endsWith("WALL")) {
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREWALL");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKWALL");
        }
        else if(parking.endsWith("CENTRAL")) {
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PRECENTRAL");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKCENTRAL");
        }
        else if(parking.contains("STORAGE")){
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREPARKSTORAGE");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKSTORAGE");
        }

        Trajectory traj0 =  drive.trajectoryBuilder(startPos)
                .strafeTo(preHubPos.vec())
                .build();
        taskList.add(new SplineMoveTask(drive, traj0));

        ParallelComboTask par1 = new ParallelComboTask();
        Trajectory traj = drive.trajectoryBuilder(preHubPos, true)
                .splineTo(hubPos.vec(), hubPos.getHeading(), velConstraints, accConstraint)
                .build();

        par1.addTask(new SplineMoveTask(drive, traj));
        par1.addTask(new LiftBucketTask(robotHardware, robotProfile, targetLiftLevel));
        taskList.add(par1);
        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(hubPos.getX(), hubPos.getY(), hubPos.getHeading() + Math.PI))
                .splineTo(duckSpinPos.vec(), duckSpinPos.getHeading(), velConstraints, accConstraint)
                .build();

        taskList.add(new SplineMoveTask(drive, traj2));
        taskList.add(new DuckCarouselSpinTask(robotHardware, startPosStr));

        if(parking.contains("STORAGE")){
            Trajectory traj3 = drive.trajectoryBuilder(duckSpinPos, true)
//                    .splineTo(afterSpinPos.vec(), afterSpinPos.getHeading() + Math.PI, velConstraints, accConstraint)
                    .splineTo(preParkPos.vec(), preHubPos.getHeading())
                    .splineTo(parkPos.vec(), parkPos.getHeading())
                    .build();
            taskList.add(new SplineMoveTask(drive, traj3));
        }else {
            Trajectory traj3 = drive.trajectoryBuilder(duckSpinPos, true)
                    .splineTo(afterSpinPos.vec(), afterSpinPos.getHeading() + Math.PI, velConstraints, accConstraint)
                    .build();
            taskList.add(new SplineMoveTask(drive, traj3));

            ParallelComboTask par2 = new ParallelComboTask();
            Trajectory traj4 = drive.trajectoryBuilder(afterSpinPos)
                    .splineTo(preParkPos.vec(), preParkPos.getHeading())
                    .splineTo(parkPos.vec(), parkPos.getHeading())
                    .build();
            par2.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
            par2.addTask(new SplineMoveTask(drive, traj4));
            taskList.add(par2);
        }
//        MecanumRotateMoveTask m1 = new MecanumRotateMoveTask(robotHardware, robotProfile);
//        m1.setRotateHeading(startPos, pos1);
//        m1.setPower(0.5);
//        taskList.add(m1);
//        MecanumRotateMoveTask m2 = new MecanumRotateMoveTask(robotHardware, robotProfile);
//        m2.setRotateHeading(pos1, duckSpinPos);
//        m2.setTimeOut(2000);
//        m2.setPower(0.3);
//        taskList.add(m2);
//        //taskList.add(new RobotSleep(3000));
//        taskList.add(new DuckCarouselSpinTask(robotHardware, startPosStr));
//        MecanumRotateMoveTask m3 = new MecanumRotateMoveTask(robotHardware, robotProfile);
//        m3.setRotateHeading(duckSpinPos, pos2);
//        m3.setPower(0.5);
//        taskList.add(m3);
//        //taskList.add(new RobotSleep(3000));
//        PIDMecanumMoveTask m4 = new PIDMecanumMoveTask(robotHardware, robotProfile);
//        m4.setPath(pos2, hubPos);
//        m4.setPower(0.5);
//        taskList.add(m4);
//        taskList.add(new LiftBucketTask(robotHardware, robotProfile, targetLiftLevel));
//        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));
//        PIDMecanumMoveTask m5 = new PIDMecanumMoveTask(robotHardware, robotProfile);
//        m5.setPath(hubPos, pos3);
//        m5.setPower(0.5);
//        taskList.add(m5);
//        PIDMecanumMoveTask m6 = new PIDMecanumMoveTask(robotHardware, robotProfile);
//        m6.setPath(pos3, parkPos);
//        m6.setPower(0.5);
//        taskList.add(m6);
//        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
    }

    public ArrayList<RobotControl> prepareSHDelivery_WarehouseParkingWall_TaskList(){
        return null;
    }

    public void prepareSHDelivery_Carousel_StorageParkingTaskList(){

    }

    public void prepareSHDelivery_WarehousePickup_SHDelivery_WarehouseParkingWallTaskList(){

    }
}