package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
    int delay_parking;
    String startingPositionModes;
    String parking;
    String deliverRoute;
    String parkOnly;
    RobotVision.AutonomousGoal goal;
    Pose2d preParkPos, parkPos;
    AngleMath.Direction direction;

    RobotHardware.LiftPosition targetLiftLevel;

    public AutonomousTaskBuilder(DriverOptions driverOptions, RobotHardware robotHardware, RobotProfile robotProfile, RobotVision.AutonomousGoal goal) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;

        this.driverOptions = driverOptions;
        this.delay = driverOptions.getDelay();
        this.startingPositionModes = driverOptions.getStartingPositionModes();
        this.parking = driverOptions.getParking();
        this.delay_parking = driverOptions.getDelayParking();
        this.deliverRoute = driverOptions.getDeliveryRoutes();
        this.parkOnly = driverOptions.getParkingOnly();
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
        if (parkOnly.equals("PARK_ONLY")){
            buildParkOnlyTasks(startingPositionModes);
        }
        else if (startingPositionModes.endsWith("DUCK")) {
             buildDuckTasks(startingPositionModes);
        }
        else if (startingPositionModes.endsWith("DEPOT")){
             buildDepotTasks(startingPositionModes);
        }
        robotHardware.getLocalizer().setPoseEstimate(startPos);
        return taskList;
    }

    public void buildParkOnlyTasks(String startPosStr){
        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        parkPos = robotProfile.getProfilePose(startPosStr + "_PARKWALL");
        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        Trajectory traj1 = drive.trajectoryBuilder(startPos, true)
                .splineTo(new Vector2d(parkPos.getX(), parkPos.getY()), parkPos.getHeading())
                .build();
        taskList.add(new SplineMoveTask(drive, traj1));
    }

    public void buildDepotTasks(String startPosStr){
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(15, 15, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(50, 50, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(11, 11, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((15));
        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d preHubPos = robotProfile.getProfilePose(startPosStr + "_PREHUB");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_HUB");
        Pose2d warehousePickupPos = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL");
        Pose2d prePickPos = robotProfile.getProfilePose(startPosStr + "_PREWALL");;
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

        // move out of the wall
        Trajectory traj0 =  drive.trajectoryBuilder(startPos)
                .strafeTo(preHubPos.vec(), fastVelConstraints, accConstraint)
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
        taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));
        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        //*
        //Between delivering to hub and parking:
        //Pick up block, drive to hub and deliver.
        Trajectory traj2 = drive.trajectoryBuilder(hubPos)
                .splineTo(prePickPos.vec(), prePickPos.getHeading(),fastVelConstraints, accConstraint)
                .splineTo(warehousePickupPos.vec(), warehousePickupPos.getHeading(), fastVelConstraints, accConstraint)
                .build();
        ParallelComboTask par2 = new ParallelComboTask();
        par2.addTask(new SplineMoveTask(drive, traj2));
        par2.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
        par2.addTask(new IntakeTask(robotHardware, robotProfile));
        taskList.add(par2);

        Trajectory traj3 = drive.trajectoryBuilder(warehousePickupPos, true)
                .splineTo(prePickPos.vec(), prePickPos.getHeading()+Math.PI, fastVelConstraints, accConstraint)
                .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI, fastVelConstraints, accConstraint)
                .build();
        ParallelComboTask par3 = new ParallelComboTask();
        par3.addTask(new SplineMoveTask(drive, traj3));
        par3.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.TOP));
        taskList.add(par3);

        taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));
        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        Trajectory traj4 = drive.trajectoryBuilder(hubPos)
                .splineTo(prePickPos.vec(), prePickPos.getHeading(), fastVelConstraints, accConstraint)
                .splineTo(warehousePickupPos.vec(), warehousePickupPos.getHeading(), fastVelConstraints, accConstraint)
                .build();
        ParallelComboTask par4 = new ParallelComboTask();
        par4.addTask(new SplineMoveTask(drive, traj4));
        par4.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
        par4.addTask(new IntakeTask(robotHardware, robotProfile));
        taskList.add(par4);

        Trajectory traj5 = drive.trajectoryBuilder(warehousePickupPos, true)
                .splineTo(prePickPos.vec(), prePickPos.getHeading()+Math.PI, fastVelConstraints, accConstraint)
                .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI)
                .build();

        ParallelComboTask par5 = new ParallelComboTask();
        par5.addTask(new SplineMoveTask(drive, traj5));
        par5.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.TOP));
        taskList.add(par5);

        taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));
        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        Trajectory traj6 = drive.trajectoryBuilder(hubPos)
                .splineTo(preParkPos.vec(), preParkPos.getHeading(), fastVelConstraints, accConstraint)
                .splineTo(parkPos.vec(), parkPos.getHeading(), fastVelConstraints, accConstraint)
                .build();
        ParallelComboTask par6 = new ParallelComboTask();
        par6.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));
        par6.addTask(new SplineMoveTask(drive, traj6));
        taskList.add(par6);
    }

    public void buildDuckTasks(String startPosStr) {
        Pose2d preParkPos_0 = null, preParkPos_1 = null;

        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(15, 15, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(11, 11, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((15));
        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d preCarouselPos = robotProfile.getProfilePose(startPosStr + "_PRECAROUSEL");
        Pose2d duckSpinPos = robotProfile.getProfilePose(startPosStr + "_CAROUSEL");
        Pose2d preHubPos1 = robotProfile.getProfilePose(startPosStr + "_PREHUB_1");
        Pose2d preHubPos2 = robotProfile.getProfilePose(startPosStr + "_PREHUB_2");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_HUB");
        Pose2d afterHubEstimatePos = robotProfile.getProfilePose(startPosStr + "_AFTER_HUB_ESTIMATE");
        boolean isDuckParkingCCW = driverOptions.isDuckParkingCCW();

        if(parking.contains("STORAGE")){
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREPARKSTORAGE");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKSTORAGE");
        }else{   //choice of park Wall or Central
            if(!isDuckParkingCCW){
                preParkPos_0 = robotProfile.getProfilePose(startPosStr + "_PRE_PARK_CW_0");
                preParkPos_1 = robotProfile.getProfilePose(startPosStr + "_PRE_PARK_CW_1");
            }else {
                preParkPos_0 = robotProfile.getProfilePose(startPosStr + "_PRE_PARK_CCW_0");
                preParkPos_1 = robotProfile.getProfilePose(startPosStr + "_PRE_PARK_CCW_1");
            }
            preParkPos = parking.contains("WALL") ? robotProfile.getProfilePose(startPosStr + "_PREWALL")
                                                  : robotProfile.getProfilePose(startPosStr + "_PRE_CENTRAL");
            parkPos = parking.contains("WALL") ? parkPos = robotProfile.getProfilePose(startPosStr + "_PARKWALL")
                                               : robotProfile.getProfilePose(startPosStr + "_PARK_CENTRAL");
        }

        Trajectory traj0 = drive.trajectoryBuilder(startPos)
                            .strafeTo(preCarouselPos.vec())
                            .build();
        taskList.add(new SplineMoveTask(drive, traj0));

        Trajectory traj1 = drive.trajectoryBuilder(preCarouselPos)
                            .splineTo(duckSpinPos.vec(), duckSpinPos.getHeading(),slowVelConstraints, accConstraint)
                            .forward(1)
                            .build();
        taskList.add(new SplineMoveTask(drive, traj1));
        taskList.add(new DuckCarouselSpinTask(robotHardware, startPosStr));

        ParallelComboTask par1 = new ParallelComboTask();
        Trajectory traj2 = drive.trajectoryBuilder(duckSpinPos, true)
                                .splineTo(preHubPos1.vec(), preHubPos1.getHeading())
                                .splineTo(preHubPos2.vec(), preHubPos2.getHeading())
                                .splineTo(hubPos.vec(), hubPos.getHeading())
                                .build();
        par1.addTask(new SplineMoveTask(drive, traj2));
        par1.addTask(new LiftBucketTask(robotHardware, robotProfile, targetLiftLevel));
        taskList.add(par1);

        taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));

        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));

        if(parking.contains("STORAGE")){
            Trajectory traj3 = drive.trajectoryBuilder(afterHubEstimatePos)
                            .splineTo(preParkPos.vec(), preParkPos.getHeading())
                            .build();
            taskList.add(new SplineMoveTask(drive, traj3));

            Trajectory traj4 = drive.trajectoryBuilder(preParkPos)
                               .strafeTo(parkPos.vec())
                               .build();
            taskList.add(new SplineMoveTask(drive, traj4));
        } else {   //wall or central parking
            Trajectory traj3 = drive.trajectoryBuilder(afterHubEstimatePos)
                    .splineTo(preParkPos_0.vec(), preParkPos_0.getHeading())
                    .splineTo(preParkPos_1.vec(), preParkPos_1.getHeading())
                    .build();
            taskList.add(new SplineMoveTask(drive, traj3));

            if (isDuckParkingCCW && delay_parking>0){
                taskList.add(new RobotSleep(delay_parking * 1000));
            }

            Trajectory traj4 = drive.trajectoryBuilder(preParkPos_1)
                    .strafeTo(preParkPos.vec())
                    .build();
            taskList.add(new SplineMoveTask(drive, traj4));
            Trajectory traj5 = drive.trajectoryBuilder(preParkPos)
                    .splineTo(parkPos.vec(), parkPos.getHeading(), fastVelConstraints, accConstraint)
                    .build();
            taskList.add(new SplineMoveTask(drive, traj5));
        }
    }
}