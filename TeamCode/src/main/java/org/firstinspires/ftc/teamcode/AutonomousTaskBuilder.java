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
    int start_delay;
    int delay_parking_storage;
    int delay_time_parking_warehouse;
    int freight_delivery_count;
    String startingPositionModes;
    String parking;
    String parkOnly;
    RobotVision.AutonomousGoal goal;
    Pose2d preParkPos, parkPos, parkCentralFarPos;
    boolean isDeliverToHubBySideRoute;

    RobotHardware.LiftPosition targetLiftLevel;

    public AutonomousTaskBuilder(DriverOptions driverOptions, RobotHardware robotHardware, RobotProfile robotProfile, RobotVision.AutonomousGoal goal) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;

        this.driverOptions = driverOptions;
        this.start_delay = driverOptions.getStartDelay();
        this.startingPositionModes = driverOptions.getStartingPositionModes();
        this.parking = driverOptions.getParking();
        this.delay_parking_storage = driverOptions.getDelayParkingByStorageFromEnding();
        this.delay_time_parking_warehouse = driverOptions.getDelayParkingByWarehouseFromEnding();
        this.parkOnly = driverOptions.getParkingOnly();
        this.freight_delivery_count = driverOptions.getFreightDeliveryCount();
        this.isDeliverToHubBySideRoute = driverOptions.isDeliverToHubBySideRoute();
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
        if (start_delay>0) {
            taskList.add(new RobotSleep(start_delay*1000));
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
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(35, 25, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(50, 50, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(10, 10, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((40));
        TrajectoryAccelerationConstraint slowAccConstraint = SampleMecanumDrive.getAccelerationConstraint((25));
        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d preHubPos = robotProfile.getProfilePose(startPosStr + "_PREHUB");
        Pose2d hubPos_0 = robotProfile.getProfilePose(startPosStr + "_HUB_0");
        Pose2d hubPos_1 = robotProfile.getProfilePose(startPosStr + "_HUB_1");
        Pose2d afterHubEstimatePos = robotProfile.getProfilePose(startPosStr + "_AFTER_HUB_ESTIMATE");
        Pose2d warehousePickupPos_0 = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL_0");
        Pose2d warehousePickupPos_1 = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL_1");
        Pose2d warehousePickupPos_2 = robotProfile.getProfilePose(startPosStr + "_PICKUPWALL_2");
        Pose2d prePickPos_0 = robotProfile.getProfilePose(startPosStr + "_PREWALL");;
        Pose2d prePickPos_1 = robotProfile.getProfilePose(startPosStr + "_PREPICK");;

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
        SequentialComboTask sc0 = new SequentialComboTask();

        ParallelComboTask par1 = new ParallelComboTask();
        Trajectory traj0 =  drive.trajectoryBuilder(startPos)
                .strafeTo(preHubPos.vec(), fastVelConstraints, accConstraint)
//                .turn(Math.toRadians(90))
                .build();
        par1.addTask(new SplineMoveTask(drive, traj0));
        par1.addTask(new LiftBucketTask(robotHardware));
        sc0.addTask(par1);

        // deliver to hub
        if(driverOptions.isDeliverToHubUsingOpencv()) {
            Trajectory traj1a = drive.trajectoryBuilder(preHubPos, true)
                                .splineTo(hubPos_1.vec(), hubPos_1.getHeading()+Math.PI, velConstraints, accConstraint)
                                .build();
            sc0.addTask(new SplineMoveTask(drive, traj1a));
            sc0.addTask(new AutoHubApproachTask(robotHardware, robotProfile));
        }else {
            Trajectory traj1b = drive.trajectoryBuilder(preHubPos, true)
//                                .splineTo(hubPos_1.vec(), hubPos_1.getHeading()+Math.PI, fastVelConstraints, accConstraint)
                                .splineToLinearHeading(afterHubEstimatePos, afterHubEstimatePos.getHeading()+Math.PI, fastVelConstraints, accConstraint)
                                .build();
            sc0.addTask(new SplineMoveTask(drive, traj1b));
        }
        sc0.addTask(new DeliverToHubTask(robotHardware, robotProfile));
        taskList.add(sc0);

        //now, 3 things going on one trip from hub to warehouse pickup.
        //(1) one movement trajectory
        //(2) start intake at certain point (using WaitForPoseTask)
        //(3) complete the trajectory whichever comes first: intake a freight or timeout in completing trajectory
        Trajectory traj2 = drive.trajectoryBuilder(afterHubEstimatePos)
                .splineTo(prePickPos_0.vec(), prePickPos_0.getHeading(), velConstraints, accConstraint)
                .splineTo(prePickPos_1.vec(), prePickPos_1.getHeading(), fastVelConstraints, accConstraint)
//                .splineTo(warehousePickupPos_0.vec(), warehousePickupPos_0.getHeading(), velConstraints, accConstraint)
                .splineTo(warehousePickupPos_1.vec(), warehousePickupPos_1.getHeading(), fastVelConstraints, slowAccConstraint)
                .build();

        SequentialComboTask sc1a = new SequentialComboTask();
        ParallelComboIntakeMovePriorityTask par2 = new ParallelComboIntakeMovePriorityTask();
        par2.addTask(new AutoIntakeTask(robotHardware, robotProfile, 6000, true));
        SequentialComboTask sc1b = new SequentialComboTask();
        sc1b.addTask(new AutoIntakeSplineMoveTask(traj2, robotHardware));
        sc1b.addTask(new RobotSleep(3000));
        par2.addTask(new WaitForPoseTask((RealSenseLocalizer) robotHardware.getLocalizer(), new Pose2d(63,-8,0), new Pose2d(106,36,0)));
        par2.addTask(sc1b);
        sc1a.addTask(par2);
        taskList.add(sc1a);

        ArrayList<Pose2d> pickupToHubPoses = new ArrayList<>();
        ArrayList<TrajectoryVelocityConstraint> poseSpeed= new ArrayList<>();

        SequentialComboTask sc2 = new SequentialComboTask();
        if(driverOptions.isDeliverToHubUsingOpencv()) {
            pickupToHubPoses.add(prePickPos_1);
            pickupToHubPoses.add(prePickPos_0);
            pickupToHubPoses.add(hubPos_1);
            poseSpeed.add(fastVelConstraints);
            poseSpeed.add(fastVelConstraints);
            poseSpeed.add(velConstraints);

            sc2.addTask(new InstantaneousPostionTrajectoryTask(robotHardware, pickupToHubPoses, poseSpeed, true));
            sc2.addTask(new AutoHubApproachTask(robotHardware, robotProfile));
        }else{
            pickupToHubPoses.add(prePickPos_1);
            pickupToHubPoses.add(prePickPos_0);
//            pickupToHubPoses.add(hubPos_0);
            pickupToHubPoses.add(hubPos_1);
//            pickupToHubPoses.add(preHubPos);
            pickupToHubPoses.add(afterHubEstimatePos);
            poseSpeed.add(fastVelConstraints);
            poseSpeed.add(fastVelConstraints);
            poseSpeed.add(fastVelConstraints);
            poseSpeed.add(velConstraints);

            sc1a.addTask(new InstantaneousPostionTrajectoryTask(robotHardware, pickupToHubPoses, poseSpeed,true));
        }
        sc2.addTask(new DeliverToHubTask(robotHardware, robotProfile));
        taskList.add(sc2);

        if(freight_delivery_count ==2) {
            Trajectory traj4 = drive.trajectoryBuilder(afterHubEstimatePos)
                                .splineTo(prePickPos_0.vec(), prePickPos_0.getHeading(), velConstraints, accConstraint)
                                .splineTo(prePickPos_1.vec(), prePickPos_1.getHeading(), fastVelConstraints, accConstraint)
//                                .splineTo(warehousePickupPos_0.vec(), warehousePickupPos_0.getHeading(), velConstraints, accConstraint)
                                .splineTo(warehousePickupPos_2.vec(), warehousePickupPos_2.getHeading(), fastVelConstraints, slowAccConstraint)
                                .build();
            SequentialComboTask sc3a = new SequentialComboTask();
            ParallelComboIntakeMovePriorityTask par3 = new ParallelComboIntakeMovePriorityTask();
            par3.addTask(new AutoIntakeTask(robotHardware, robotProfile, 6000, true));
            SequentialComboTask sc3b = new SequentialComboTask();
            sc3b.addTask(new AutoIntakeSplineMoveTask(traj4, robotHardware));
            sc3b.addTask(new RobotSleep(3000));
            par3.addTask(new WaitForPoseTask((RealSenseLocalizer) robotHardware.getLocalizer(), new Pose2d(63,-8,0), new Pose2d(106,36,0)));
            par3.addTask(sc3b);
            sc3a.addTask(par3);
            taskList.add(sc3a);

            // deliver to hub
            pickupToHubPoses.clear();
            SequentialComboTask sc4 = new SequentialComboTask();
            if(driverOptions.isDeliverToHubUsingOpencv()) {
                pickupToHubPoses.add(prePickPos_1);
                pickupToHubPoses.add(prePickPos_0);
//                pickupToHubPoses.add(hubPos_0);
                pickupToHubPoses.add(hubPos_1);
                sc4.addTask(new InstantaneousPostionTrajectoryTask(robotHardware, pickupToHubPoses, null,true));
                sc4.addTask(new AutoHubApproachTask(robotHardware, robotProfile));
            }else {
                pickupToHubPoses.add(prePickPos_1);
                pickupToHubPoses.add(prePickPos_0);
//                pickupToHubPoses.add(hubPos_0);
                pickupToHubPoses.add(hubPos_1);
//                pickupToHubPoses.add(preHubPos);
                pickupToHubPoses.add(afterHubEstimatePos);
                sc4.addTask(new InstantaneousPostionTrajectoryTask(robotHardware, pickupToHubPoses, null,true));
            }
            sc4.addTask(new DeliverToHubTask(robotHardware, robotProfile));
            taskList.add(sc4);
        }

        Trajectory traj6 = drive.trajectoryBuilder(afterHubEstimatePos)
                .splineTo(preParkPos.vec(), preParkPos.getHeading(), velConstraints, accConstraint)
                .splineTo(parkPos.vec(), parkPos.getHeading(), fastVelConstraints, slowAccConstraint)
                .build();
        ParallelComboTask par6 = new ParallelComboTask();
        par6.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ONE));
        par6.addTask(new SplineMoveTask(drive, traj6));
        taskList.add(par6);
    }

    public void buildDuckTasks(String startPosStr) {
        Pose2d preParkPos_0 = null, preParkPos_1 = null;

        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(50, 50, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(11, 11, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((40));
        TrajectoryAccelerationConstraint slowAccConstraint = SampleMecanumDrive.getAccelerationConstraint((20));
        TrajectoryAccelerationConstraint fastAccConstraint = SampleMecanumDrive.getAccelerationConstraint((80));

        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d preCarouselPos = robotProfile.getProfilePose(startPosStr + "_PRECAROUSEL");
        Pose2d duckSpinPos = robotProfile.getProfilePose(startPosStr + "_CAROUSEL");
        Pose2d hubPos_0 = robotProfile.getProfilePose(startPosStr + "_HUB_0");
        Pose2d hubPos_1 = robotProfile.getProfilePose(startPosStr + "_HUB_1");
        Pose2d intakePos_0 = robotProfile.getProfilePose(startPosStr + "_INTAKE_0");
        Pose2d intakePos_1 = robotProfile.getProfilePose(startPosStr + "_INTAKE_1");
        Pose2d afterHubEstimatePos = robotProfile.getProfilePose(startPosStr + "_AFTER_HUB_ESTIMATE");

        Pose2d parking_delay_warehousePos = robotProfile.getProfilePose(startPosStr + "_PARKING_DELAY_WAREHOUSE");
        Pose2d parking_delay_storage = robotProfile.getProfilePose(startPosStr + "_PARKING_DELAY_STORAGE");

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
            parkCentralFarPos = parking.contains("WALL") ? null
                                                : robotProfile.getProfilePose(startPosStr + "_PARK_CENTRAL_FAR");

        }

        SequentialComboTask sc0 = new SequentialComboTask();
        ParallelComboTask par1 = new ParallelComboTask();
        Trajectory traj0 = drive.trajectoryBuilder(startPos)
                            .strafeTo(preCarouselPos.vec())
                            .build();
        par1.addTask(new SplineMoveTask(drive, traj0));
        par1.addTask(new LiftBucketTask(robotHardware));
        sc0.addTask(par1);

        Trajectory traj1 = drive.trajectoryBuilder(preCarouselPos)
                            .splineTo(duckSpinPos.vec(), duckSpinPos.getHeading(), velConstraints, slowAccConstraint)
                            .forward(1)
                            .build();
        sc0.addTask(new DuckSplineMoveTask(drive, traj1, robotHardware));
        sc0.addTask(new DuckCarouselSpinTask(robotHardware, startPosStr));
        taskList.add(sc0);

        SequentialComboTask sc1 = new SequentialComboTask();
        if(driverOptions.isDeliverToHubUsingOpencv()) {
            Trajectory traj2 = drive.trajectoryBuilder(duckSpinPos, true)
                    .splineTo(hubPos_1.vec(), hubPos_1.getHeading()+Math.PI, fastVelConstraints, accConstraint)
                    .build();
            sc1.addTask(new SplineMoveTask(drive, traj2));
            sc1.addTask(new AutoHubApproachTask(robotHardware, robotProfile));
        }else{
            Trajectory traj2 = drive.trajectoryBuilder(duckSpinPos, true)
                    .splineToSplineHeading(afterHubEstimatePos, afterHubEstimatePos.getHeading()+Math.PI, velConstraints, accConstraint)
                    .build();
            sc1.addTask(new SplineMoveTask(drive, traj2));
        }

        sc1.addTask(new DeliverToHubTask(robotHardware, robotProfile));
        taskList.add(sc1);

        SequentialComboTask sc2 = new SequentialComboTask();
        if(parking.contains("STORAGE")){
            Trajectory traj3 = drive.trajectoryBuilder(afterHubEstimatePos)
                            .splineTo(preParkPos.vec(), preParkPos.getHeading(), fastVelConstraints, accConstraint)
                            .build();
            ParallelComboTask par0 = new ParallelComboTask();
            par0.addTask(new SplineMoveTask(drive, traj3));
            par0.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ONE));
            sc2.addTask(par0);

            Trajectory traj4 = drive.trajectoryBuilder(preParkPos)
                               .strafeTo(parkPos.vec())
                               .build();
            sc2.addTask(new SplineMoveTask(drive, traj4));
        } else {   //wall or central parking
            Trajectory traj3;
            Pose2d nextPos = afterHubEstimatePos;

            if (isDuckParkingCCW && delay_parking_storage>0){
                Trajectory traj3a = drive.trajectoryBuilder(afterHubEstimatePos)
                                    .splineTo(parking_delay_storage.vec(), parking_delay_storage.getHeading(), velConstraints, accConstraint)
                                    .build();
                sc2.addTask(new SplineMoveTask(drive, traj3a));
                sc2.addTask(new RobotSleep(delay_parking_storage * 1000));
                nextPos = parking_delay_storage;
            }

            if(!isDuckParkingCCW){  //cw
                traj3 = drive.trajectoryBuilder(nextPos)
                                .splineTo(preParkPos_0.vec(), preParkPos_0.getHeading(), velConstraints, accConstraint)
                                .splineTo(preParkPos_1.vec(), preParkPos_1.getHeading(),fastVelConstraints, accConstraint)
                                .build();
                sc2.addTask(new SplineMoveTask(drive, traj3));

                if(delay_time_parking_warehouse >0){
                    Trajectory traj4 = drive.trajectoryBuilder(preParkPos_1)
                                        .strafeTo(parking_delay_warehousePos.vec(), velConstraints, accConstraint)
                                        .build();
                    sc2.addTask(new SplineMoveTask(drive, traj4));
                    sc2.addTask(new RobotSleep(delay_time_parking_warehouse *1000));
                    Trajectory traj5 = drive.trajectoryBuilder(parking_delay_warehousePos)
                                        .strafeTo(preParkPos.vec())
                                        .build();
                    sc2.addTask(new SplineMoveTask(drive, traj5));
                    Trajectory traj6 = drive.trajectoryBuilder(preParkPos)
                                        .splineTo(parkPos.vec(), parkPos.getHeading(), fastVelConstraints, accConstraint)
                                        .build();
                    sc2.addTask(new SplineMoveTask(drive, traj6));
                }else {  //no warehouse delay
                    Trajectory traj4 = drive.trajectoryBuilder(preParkPos_1)
                            .strafeTo(preParkPos.vec())
                            .build();
                    sc2.addTask(new SplineMoveTask(drive, traj4));

                    Trajectory traj5 = drive.trajectoryBuilder(preParkPos)
                            .splineTo(parkPos.vec(), parkPos.getHeading(), fastVelConstraints, accConstraint)
                            .build();
                    sc2.addTask(new SplineMoveTask(drive, traj5));

                    Trajectory traj6 = drive.trajectoryBuilder(parkPos)
                            .splineTo(intakePos_0.vec(), intakePos_0.getHeading(), slowVelConstraints, accConstraint)
                            .splineTo(intakePos_1.vec(), intakePos_1.getHeading(), slowVelConstraints, accConstraint)
                            .build();
                    ParallelComboIntakeMovePriorityTask par2 = new ParallelComboIntakeMovePriorityTask();
                    par2.addTask(new AutoIntakeTask(robotHardware, robotProfile, 6000, true));
                    SequentialComboTask sc3 = new SequentialComboTask();
                    sc3.addTask(new AutoIntakeSplineMoveTask(traj6, robotHardware));
                    sc3.addTask(new RobotSleep(3000));
                    par2.addTask(sc3);
                    par2.addTask(new WaitForPoseTask((RealSenseLocalizer) robotHardware.getLocalizer(), new Pose2d(63,-8,0), new Pose2d(106,36,0)));
                    sc2.addTask(par2);
                    if(parkCentralFarPos!=null){
                        sc2.addTask(new InstantaneousPostionTrajectoryTask(robotHardware, parkCentralFarPos, true));
                    }else{
                        sc2.addTask(new InstantaneousPostionTrajectoryTask(robotHardware, parkPos, true));
                    }
                    sc2.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.BOTTOM));
                    taskList.add(sc2);
                }
            } else {  //ccw
                ParallelComboTask parNoT265_a = new ParallelComboTask();
                parNoT265_a.addTask(new T265IgnoreConfidenceLevelTask(robotHardware, true));
                Trajectory traj3a = drive.trajectoryBuilder(nextPos)
                                    .splineTo(preParkPos_0.vec(), preParkPos_0.getHeading(),fastVelConstraints, fastAccConstraint)
                                    .build();
                sc2.addTask(new SplineMoveTask(drive, traj3a));

                Trajectory traj3b = drive.trajectoryBuilder(preParkPos_0)
                                    .splineTo(preParkPos_1.vec(), preParkPos_1.getHeading(), fastVelConstraints, accConstraint)
                                    .splineTo(preParkPos.vec(), preParkPos.getHeading(), fastVelConstraints, accConstraint)
                                    .splineToLinearHeading(parkPos, parkPos.getHeading(), velConstraints, accConstraint)
                                    .build();
                sc2.addTask(new SplineMoveTask(drive, traj3b));
                parNoT265_a.addTask(sc2);
                taskList.add(parNoT265_a);

                Trajectory traj4 = drive.trajectoryBuilder(parkPos)
                        .splineTo(intakePos_0.vec(), intakePos_0.getHeading(), velConstraints,accConstraint)
                        .splineTo(intakePos_1.vec(), intakePos_1.getHeading(), slowVelConstraints,accConstraint)
                        .build();

                ParallelComboIntakeMovePriorityTask par2 = new ParallelComboIntakeMovePriorityTask();
                par2.addTask(new T265IgnoreConfidenceLevelTask(robotHardware,true));
                par2.addTask(new AutoIntakeTask(robotHardware, robotProfile, 6000, true));
                SequentialComboTask sc3 = new SequentialComboTask();
                sc3.addTask(new AutoIntakeSplineMoveTask(traj4, robotHardware));
                sc3.addTask(new RobotSleep(3000));
                par2.addTask(sc3);
                par2.addTask(new WaitForPoseTask((RealSenseLocalizer) robotHardware.getLocalizer(), new Pose2d(63,-8,0), new Pose2d(106,36,0)));
                sc2.addTask(par2);

                Pose2d prehub2 = new Pose2d(37,30,320);
                Pose2d secondHub = new Pose2d(33,37,320);
                ArrayList<Pose2d> backToHubPoses = new ArrayList<>();
                ArrayList<TrajectoryVelocityConstraint> posesSpeed = new ArrayList<>();
//                ParallelComboTask par3 = new ParallelComboTask();
//                par3.addTask(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.TOP));
//                par3.addTask(new InstantaneousPostionTrajectoryTask(robotHardware,preParkPos, true ));
//                sc2.addTask(par3);

                //this probably will be replaced by autoApproach:
                ParallelComboTask parNoT265_b = new ParallelComboTask();
                parNoT265_b.addTask(new T265IgnoreConfidenceLevelTask(robotHardware, true));

                SequentialComboTask sc4 = new SequentialComboTask();
                if(driverOptions.isDeliverToHubUsingOpencv()){
                    backToHubPoses.add(parkPos);
                    backToHubPoses.add(preParkPos);
                    backToHubPoses.add(prehub2);
                    posesSpeed.add(fastVelConstraints);
                    posesSpeed.add(velConstraints);
                    posesSpeed.add(slowVelConstraints);
//                    Trajectory traj5 = drive.trajectoryBuilder(preParkPos, true)
//                            .splineTo(prehub2.vec(), Math.toRadians(prehub2.getHeading())+Math.PI, slowVelConstraints, accConstraint)
//                            .build();
//                    sc4.addTask(new SplineMoveTask(drive, traj5));
                    sc4.addTask(new InstantaneousPostionTrajectoryTask(robotHardware,backToHubPoses, posesSpeed, true ));
                    sc4.addTask(new InstantaneousPostionTrajectoryTask(robotHardware,secondHub, true ));
                    sc4.addTask(new AutoHubApproachTask(robotHardware, robotProfile));
                }else {
                    backToHubPoses.add(parkPos);
                    backToHubPoses.add(preParkPos);
                    backToHubPoses.add(prehub2);
                    backToHubPoses.add(secondHub);

                    posesSpeed.add(fastVelConstraints);
                    posesSpeed.add(slowVelConstraints);
                    posesSpeed.add(fastVelConstraints);
                    posesSpeed.add(velConstraints);

//                    Trajectory traj5 = drive.trajectoryBuilder(preParkPos, true)
//                            .splineTo(secondHub.vec(), Math.toRadians(secondHub.getHeading()) + Math.PI, velConstraints, accConstraint)
//                            .build();
//                    sc4.addTask(new SplineMoveTask(drive, traj5));
                    sc4.addTask(new InstantaneousPostionTrajectoryTask(robotHardware,backToHubPoses, posesSpeed, true ));
                }
                sc4.addTask(new DeliverToHubTask(robotHardware, robotProfile));
                parNoT265_b.addTask(sc4);
                taskList.add(parNoT265_b);

                Trajectory traj6 = drive.trajectoryBuilder(secondHub)
                        .splineTo(preParkPos.vec(), preParkPos.getHeading(), fastVelConstraints, accConstraint)
                        .splineToLinearHeading(parkPos, parkPos.getHeading(), fastVelConstraints, accConstraint)
                        .build();
                taskList.add(new SplineMoveTask(drive, traj6));
            }
        }
    }
}