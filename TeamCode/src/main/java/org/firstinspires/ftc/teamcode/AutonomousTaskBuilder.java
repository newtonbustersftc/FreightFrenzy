package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
    Pose2d startPos = new Pose2d();
    int delay;
    int delay_parking;
    int freight_delivery_count;
    String startingPositionModes;
    String parking;
    String parkOnly;
    RobotVision.AutonomousGoal goal;
    Pose2d preParkPos, parkPos;

    RobotHardware.LiftPosition targetLiftLevel;

    public AutonomousTaskBuilder(DriverOptions driverOptions, RobotHardware robotHardware, RobotProfile robotProfile, RobotVision.AutonomousGoal goal) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;

        this.driverOptions = driverOptions;
        this.delay = driverOptions.getDelay();
        this.startingPositionModes = driverOptions.getStartingPositionModes();
        this.parking = driverOptions.getParking();
        this.delay_parking = driverOptions.getDelayParking();
        this.parkOnly = driverOptions.getParkingOnly();
        this.freight_delivery_count = driverOptions.getFreightDeliveryCount();
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

        if (parkOnly.equals("PARK_ONLY")){
            buildParkOnlyTasks(startingPositionModes);
        }
        else if (startingPositionModes.contains("DUCK")) {
             buildDuckTasks(startingPositionModes);
        }
        else if (startingPositionModes.contains("DEPOT")){
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
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(50, 50, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(10, 10, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((40));
        TrajectoryAccelerationConstraint slowAccConstraint = SampleMecanumDrive.getAccelerationConstraint((25));
        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d preHubPos = robotProfile.getProfilePose(startPosStr + "_PREHUB");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_HUB");
        Pose2d afterHubEstimatePos = robotProfile.getProfilePose(startPosStr + "_AFTER_HUB_ESTIMATE");
        Pose2d warehousePickupPos_0 = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL_0");
        Pose2d warehousePickupPos_1 = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL_1");
        Pose2d warehousePickupPos_2 = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL_2");
        Pose2d prePickPos = robotProfile.getProfilePose(startPosStr + "_PREWALL");;
        if (parking.contains("WALL")) {
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREWALL");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKWALL");
        }
        else if(parking.contains("CENTRAL")){
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PRECENTRAL");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKCENTRAL");
        }
        else { //if(parking.contains("STORAGE")) {
            preParkPos = robotProfile.getProfilePose(startPosStr + "_PREPARKSTORAGE");
            parkPos = robotProfile.getProfilePose(startPosStr + "_PARKSTORAGE");
        }

        // move out of the wall
        ParallelComboTask par1 = new ParallelComboTask();
        Trajectory traj0 =  drive.trajectoryBuilder(startPos)
                .strafeTo(preHubPos.vec(), fastVelConstraints, accConstraint)
                .build();
        par1.addTask(new SplineMoveTask(drive, traj0));
        par1.addTask(new LiftBucketTask(robotHardware));
        taskList.add(par1);

        // deliver to hub
        if(driverOptions.isDeliver_to_hub_using_opencv()) {
            Trajectory traj1 = drive.trajectoryBuilder(preHubPos, true)
                    .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI, velConstraints, accConstraint)
                    .build();
            taskList.add(new SplineMoveTask(drive, traj1));
            taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));
        }else{
            Trajectory traj1a = drive.trajectoryBuilder(preHubPos, true)
                    .splineToLinearHeading(afterHubEstimatePos, afterHubEstimatePos.getHeading()+Math.PI, velConstraints, accConstraint)
                    .build();
            taskList.add(new SplineMoveTask(drive, traj1a));
        }

        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        //*
        //Between delivering to hub and parking:Pick up block, drive to hub and deliver.
        Trajectory traj2 = drive.trajectoryBuilder(afterHubEstimatePos)
                .splineTo(prePickPos.vec(), prePickPos.getHeading(),velConstraints, accConstraint)
                .splineTo(warehousePickupPos_0.vec(), warehousePickupPos_0.getHeading(), fastVelConstraints, accConstraint)
                .build();
        ParallelComboIntakeMovePriorityTask par2 = new ParallelComboIntakeMovePriorityTask();
        par2.addTask(new AutoIntakeTask(robotHardware, robotProfile, 5000, prePickPos));
        par2.addTask(new AutoIntakeSplineMoveTask(traj2, robotHardware, prePickPos));
        taskList.add(par2);

        if(driverOptions.isDeliver_to_hub_using_opencv()) {
            Trajectory traj3 = drive.trajectoryBuilder(prePickPos, true)
                    .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI, velConstraints, slowAccConstraint)
                    .build();
            taskList.add(new SplineMoveTask(drive, traj3));
            taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));
        }else{
            Trajectory traj3 = drive.trajectoryBuilder(prePickPos, true)
                    .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI, velConstraints, accConstraint)
                    .splineToLinearHeading(afterHubEstimatePos, afterHubEstimatePos.getHeading()+Math.PI, slowVelConstraints, accConstraint)
                    .build();
            taskList.add(new SplineMoveTask(drive, traj3));
        }
        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        if(freight_delivery_count ==2) {
            Trajectory traj4 = drive.trajectoryBuilder(afterHubEstimatePos)
                    .splineTo(prePickPos.vec(), prePickPos.getHeading(), velConstraints, accConstraint)
                    .splineTo(warehousePickupPos_0.vec(), warehousePickupPos_0.getHeading(), fastVelConstraints, accConstraint)
                    .splineTo(warehousePickupPos_2.vec(), warehousePickupPos_2.getHeading(), slowVelConstraints, slowAccConstraint)
                    .build();
//            ParallelComboTask par4 = new ParallelComboTask();
            ParallelComboIntakeMovePriorityTask par4 = new ParallelComboIntakeMovePriorityTask();
            par4.addTask(new AutoIntakeSplineMoveTask(traj4, robotHardware, prePickPos));
            par4.addTask(new AutoIntakeTask(robotHardware, robotProfile, 6000, prePickPos));
            taskList.add(par4);

            if(driverOptions.isDeliver_to_hub_using_opencv()) {
                Trajectory traj5 = drive.trajectoryBuilder(prePickPos, true)
                        .splineTo(hubPos.vec(), hubPos.getHeading() + Math.PI, velConstraints, accConstraint)
                        .build();
                taskList.add(new SplineMoveTask(drive, traj5));
                taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));
            }else{
                Trajectory traj5 = drive.trajectoryBuilder(prePickPos, true)
                        .splineTo(hubPos.vec(), hubPos.getHeading() + Math.PI, velConstraints, accConstraint)
                        .splineToLinearHeading(afterHubEstimatePos, afterHubEstimatePos.getHeading()+Math.PI, slowVelConstraints, slowAccConstraint)
                        .build();
                taskList.add(new SplineMoveTask(drive, traj5));
            }
            taskList.add(new DeliverToHubTask(robotHardware, robotProfile));
        }

        Trajectory traj6 = drive.trajectoryBuilder(afterHubEstimatePos)
                .splineTo(preParkPos.vec(), preParkPos.getHeading(), velConstraints, accConstraint)
                .splineTo(parkPos.vec(), parkPos.getHeading(), fastVelConstraints, slowAccConstraint)
                .build();
        ParallelComboTask par6 = new ParallelComboTask();
        par6.addTask(new SplineMoveTask(drive, traj6));
        par6.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ONE));
        taskList.add(par6);


    }

    public void buildDuckTasks(String startPosStr) {
        Pose2d preParkPos_0 = null, preParkPos_1 = null;

        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(50, 50, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(11, 11, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((40));
        TrajectoryAccelerationConstraint slowAccConstraint = SampleMecanumDrive.getAccelerationConstraint((20));
        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d preCarouselPos = robotProfile.getProfilePose(startPosStr + "_PRECAROUSEL");
        Pose2d duckSpinPos = robotProfile.getProfilePose(startPosStr + "_CAROUSEL");
//        Pose2d preHubPos1 = robotProfile.getProfilePose(startPosStr + "_PREHUB_1");
//        Pose2d preHubPos2 = robotProfile.getProfilePose(startPosStr + "_PREHUB_2");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_HUB");
        Pose2d intakePos = robotProfile.getProfilePose(startPosStr + "_INTAKE");
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

        ParallelComboTask par1 = new ParallelComboTask();
        Trajectory traj0 = drive.trajectoryBuilder(startPos)
                            .strafeTo(preCarouselPos.vec())
                            .build();
        par1.addTask(new SplineMoveTask(drive, traj0));
        par1.addTask(new LiftBucketTask(robotHardware));
        taskList.add(par1);

        Trajectory traj1 = drive.trajectoryBuilder(preCarouselPos)
                            .splineTo(duckSpinPos.vec(), duckSpinPos.getHeading(), velConstraints, slowAccConstraint)
                            .forward(1)
                            .build();
        taskList.add(new DuckSplineMoveTask(drive, traj1, robotHardware));
        taskList.add(new DuckCarouselSpinTask(robotHardware, startPosStr));

        if(driverOptions.isDeliver_to_hub_using_opencv()) {
            Trajectory traj2 = drive.trajectoryBuilder(duckSpinPos, true)
                        .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI)
                        .build();
            taskList.add(new SplineMoveTask(drive, traj2));
            taskList.add(new AutoHubApproachTask(robotHardware, robotProfile));
        }else{
            Trajectory traj2 = drive.trajectoryBuilder(duckSpinPos, true)
                    .splineTo(hubPos.vec(), hubPos.getHeading()+Math.PI)
                    .splineToLinearHeading(afterHubEstimatePos, afterHubEstimatePos.getHeading()+Math.PI)
                    .build();
            taskList.add(new SplineMoveTask(drive, traj2));
        }

        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ONE));

        if(parking.contains("STORAGE")){
            Trajectory traj3 = drive.trajectoryBuilder(afterHubEstimatePos)
                            .splineTo(preParkPos.vec(), preParkPos.getHeading(), fastVelConstraints, accConstraint)
                            .build();
            taskList.add(new SplineMoveTask(drive, traj3));

            Trajectory traj4 = drive.trajectoryBuilder(preParkPos)
                               .strafeTo(parkPos.vec())
                               .build();
            taskList.add(new SplineMoveTask(drive, traj4));
        } else {   //wall or central parking
            Trajectory traj3;

            if (isDuckParkingCCW && delay_parking>0){
                taskList.add(new RobotSleep(delay_parking * 1000));
            }

            if(!isDuckParkingCCW){
                traj3 = drive.trajectoryBuilder(afterHubEstimatePos)
                                .splineTo(preParkPos_0.vec(), preParkPos_0.getHeading(), velConstraints, accConstraint)
                                .splineTo(preParkPos_1.vec(), preParkPos_1.getHeading(),fastVelConstraints, accConstraint)
                                .build();
                taskList.add(new SplineMoveTask(drive, traj3));

                Trajectory traj4 = drive.trajectoryBuilder(preParkPos_1)
                                    .strafeTo(preParkPos.vec())
                                    .build();
                taskList.add(new SplineMoveTask(drive, traj4));

                Trajectory traj6 = drive.trajectoryBuilder(preParkPos)
                        .splineTo(parkPos.vec(), parkPos.getHeading(), fastVelConstraints,accConstraint)
                        .build();
                taskList.add(new SplineMoveTask(drive, traj6));

                Trajectory traj7 = drive.trajectoryBuilder(parkPos)
                        .splineTo(intakePos.vec(), intakePos.getHeading(), slowVelConstraints,accConstraint)
                        .build();
                ParallelComboIntakeMovePriorityTask par4 = new ParallelComboIntakeMovePriorityTask();
                par4.addTask(new AutoIntakeTask(robotHardware, robotProfile, 4500, parkPos));
                par4.addTask(new AutoIntakeSplineMoveTask(traj7, robotHardware, parkPos));
                taskList.add(par4);
            } else {
                traj3 = drive.trajectoryBuilder(afterHubEstimatePos)
                        //.splineTo(preParkPos_0.vec(), preParkPos_0.getHeading(), velConstraints, accConstraint)
                        .splineTo(preParkPos_1.vec(), preParkPos_1.getHeading(),velConstraints, accConstraint)
                        .splineTo(preParkPos.vec(), preParkPos.getHeading(), velConstraints, accConstraint)
                        .splineToLinearHeading(parkPos, parkPos.getHeading(), fastVelConstraints, accConstraint)
                        .build();
                taskList.add(new SplineMoveTask(drive, traj3));

                Trajectory traj6 = drive.trajectoryBuilder(parkPos)
                                    .splineTo(intakePos.vec(), intakePos.getHeading(), slowVelConstraints,accConstraint)
                                    .build();
                ParallelComboIntakeMovePriorityTask par4 = new ParallelComboIntakeMovePriorityTask();
                par4.addTask(new AutoIntakeTask(robotHardware, robotProfile, 4500, parkPos));
                par4.addTask(new AutoIntakeSplineMoveTask(traj6, robotHardware, parkPos));
                taskList.add(par4);

                Pose2d prehub = new Pose2d(50,30,0);
                Pose2d secondHub = new Pose2d(33,30,-45);

                Trajectory traj7 = drive.trajectoryBuilder(parkPos, true)
    //                    .splineTo(parkPos.vec(), parkPos.getHeading()+Math.PI, velConstraints, accConstraint)
    //                    .splineTo(preParkPos.vec(), preParkPos.getHeading()+Math.PI, velConstraints, accConstraint)
                        .splineTo(prehub.vec(), prehub.getHeading()+Math.PI, velConstraints, slowAccConstraint)
    //                    .splineTo(afterHubEstimatePos.vec(), afterHubEstimatePos.getHeading()+Math.PI, velConstraints, accConstraint)
                        .splineTo(secondHub.vec(), secondHub.getHeading()+Math.PI, velConstraints, accConstraint)
                        .build();
                taskList.add(new SplineMoveTask(drive, traj7));
                taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

                Trajectory traj8 = drive.trajectoryBuilder(secondHub)
                        //.splineTo(preParkPos_0.vec(), preParkPos_0.getHeading(), velConstraints, accConstraint)
    //                    .splineTo(preParkPos_1.vec(), preParkPos_1.getHeading(),velConstraints, accConstraint)
                        .splineTo(preParkPos.vec(), preParkPos.getHeading(), velConstraints, accConstraint)
                        .splineToLinearHeading(parkPos, parkPos.getHeading(), fastVelConstraints, accConstraint)
                        .build();
                taskList.add(new SplineMoveTask(drive, traj8));
            }
        }
    }
}