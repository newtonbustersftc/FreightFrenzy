package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.lifecycle.Lifecycle;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
    ArrayList<RobotControl> taskList = new ArrayList<>();
    DriverOptions driverOptions;
    PIDMecanumMoveTask lastMovement;
    SampleMecanumDrive drive;
    Pose2d startPos;
    int delay;
    String startingPositionModes;
    String parking;
    String deliverRoute;
    RobotVision.AutonomousGoal goal;
    int shipping_hub_x,shipping_hub_y;
    double hub_heading;
    AngleMath.Direction direction;

    //after testing, the below need to move to profile:
    int starting_x = 0;
    int starting_y = 0;
    int red_left_starting_heading = 0;
    int red_right_starting_heading = 0;
    int blue_left_starting_heading = 0;
    int blue_right_starting_heading = 0;

    int red_shipping_hub_x = 150;
    int red_shipping_hub_y = 120;
    int red_storage_x = 30;
    int red_storage_y = 88;
    int red_warehouse_x = 333;
    int red_warehouse_y = 25;
    int red_warehouse_parking_central_x = 292;
    int red_warehouse_parking_central_y = 76;
    int red_delivery_route_central_y = 76;
    int red_delivery_route_wall_y = 25;
    int red_warehouse_parking_wall_y = 25;
    int red_warehouse_parking_wall_x = 292;
    double red_left_angle_to_hub = 30;  //closewise from y axis
    double red_right_angle_to_hub = 30;  //anticlockwise from y axis

    int red_carousel_x = -22;
    int red_carousel_y = 0;
    int red_carousel_heading = 15;

    int blue_shipping_hub_x = 150;
    int blue_shipping_hub_y = 240;
    int blue_storage_x = 30;
    int blue_storage_y = 270;
    int blue_warehouse_x = 333;
    int blue_warehouse_y = 333;
    int blue_warehouse_parking_central_x = 292;
    int blue_warehouse_parking_central_y = 282;
    int blue_delivery_route_central_y = 282;
    int blue_delivery_route_wall_y = 333;
    int blue_warehouse_parking_wall_y = 333;
    int blue_warehouse_parking_wall_x = 292;
    int blue_right_angle_to_hub = 30; //anticlockwise from y axis
    int blue_left_angle_to_hub = 30;  //clockwise from y axis

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
            case RIGHT:
                this.targetLiftLevel = RobotHardware.LiftPosition.TOP;
                break;
            case MIDDLE:
                this.targetLiftLevel = RobotHardware.LiftPosition.MIDDLE;
                break;
            default:
                this.targetLiftLevel = RobotHardware.LiftPosition.BOTTOM;
                break;
        }
    }

    public ArrayList<RobotControl> buildTaskList(RobotVision.AutonomousGoal goal) {
        //TODO - switch for start up position
        ArrayList<RobotControl> taskList1 = null;
        if (startingPositionModes.endsWith("DUCK")) {
             taskList1 = buildDuckTasks(startingPositionModes);
        }
        else {
             taskList1 = buildDepotTasks(startingPositionModes);
            //taskList = new ArrayList<>();   // TODO: create buildDepotTasks
        }
        robotHardware.getLocalizer().setPoseEstimate(startPos);
        return taskList1;
    }

    public ArrayList<RobotControl> buildDepotTasks(String startPosStr){

        drive = (SampleMecanumDrive)robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d pos1 = robotProfile.getProfilePose(startPosStr + "_1");
        Pose2d pos2 = robotProfile.getProfilePose(startPosStr + "_2");
        Pose2d pos3 = robotProfile.getProfilePose(startPosStr + "_3");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_HUB");
        Pose2d warehousePos = robotProfile.getProfilePose(startPosStr + "_WAREHOUSE");
        //Pose2d parkPos = robotProfile.getProfilePose(startPosStr + "_PARK");
        PIDMecanumMoveTask m1 = new PIDMecanumMoveTask(robotHardware, robotProfile);
        m1.setPath(startPos, pos1);
        m1.setPower(0.5);
        taskList.add(m1);
        MecanumRotateMoveTask m2 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        m2.setRotateHeading(pos1, hubPos);
        m2.setPower(0.5);
        taskList.add(m2);
//        MecanumRotateMoveTask m2 = new MecanumRotateMoveTask(robotHardware, robotProfile);
//        m2.setRotateHeading(pos1, hubPos);
//        m2.setPower(0.3);
//        taskList.add(m2);
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, targetLiftLevel));
        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, RobotHardware.LiftPosition.ZERO));

        MecanumRotateMoveTask m3 = new MecanumRotateMoveTask(robotHardware, robotProfile);
        m3.setRotateHeading(hubPos, startPos);
        m3.setPower(0.3);
        taskList.add(m3);

        PIDMecanumMoveTask m5 = new PIDMecanumMoveTask(robotHardware, robotProfile);
        m5.setPath(startPos, warehousePos);
         taskList.add(m5);

//        MecanumRotateMoveTask m3 = new MecanumRotateMoveTask(robotHardware, robotProfile);
//        m3.setRotateHeading(hubPos, pos2);
//        m3.setPower(0.3);
//        taskList.add(m3);
//
//        PIDMecanumMoveTask m4 = new PIDMecanumMoveTask(robotHardware, robotProfile);
//        m4.setPath(pos2, pos3);
//        m4.setPower(0.3);
//        taskList.add(m4);
//
//        PIDMecanumMoveTask m5 = new PIDMecanumMoveTask(robotHardware, robotProfile);
//        m5.setPath(pos3, warehousePos);
//        m5.setPower(0.3);
//        taskList.add(m5);

        //taskList.add(new IntakeTask(robotHardware, robotProfile));


//        //taskList.add(new RobotSleep(3000));
//        PIDMecanumMoveTask m4 = new PIDMecanumMoveTask(robotHardware, robotProfile);
//        m4.setPath(pos2, hubPos);
//        m4.setPower(0.5);
//        taskList.add(m4);
//        taskList.add(new LiftBucketTask(robotHardware, robotProfile, targetLiftLevel));
//        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));
//        PIDMecanumMoveTask m5 = new PIDMecanumMoveTask(robotHardware, robotProfile);
//        m4.setPath(hubPos, parkPos);
//        taskList.add(m4);
        return taskList;
    }

    public ArrayList<RobotControl> buildDuckTasks(String startPosStr){

        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(15, 15, 10.25);
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((15));
        drive = (SampleMecanumDrive) robotHardware.getMecanumDrive();

        startPos = robotProfile.getProfilePose(startPosStr + "_START");
        Pose2d p1 = robotProfile.getProfilePose(startPosStr + "_1");
        Pose2d duckSpinPos = robotProfile.getProfilePose(startPosStr + "_CAROUSEL");
//        Pose2d pos2 = robotProfile.getProfilePose(startPosStr + "_2");
        Pose2d hubPos = robotProfile.getProfilePose(startPosStr + "_HUB");
//        Pose2d pos3 = robotProfile.getProfilePose(startPosStr + "_3");
        Pose2d parkPos = robotProfile.getProfilePose(startPosStr + "_PARK");

        Trajectory traj = drive.trajectoryBuilder(startPos, true)
                .splineTo(new Vector2d(hubPos.getX(), hubPos.getY()), hubPos.getHeading(), velConstraints, accConstraint)
                .build();

        taskList.add(new SplineMoveTask(drive, traj));
        taskList.add(new LiftBucketTask(robotHardware, robotProfile, targetLiftLevel));
        taskList.add(new DeliverToHubTask(robotHardware, robotProfile));

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(hubPos.getX(), hubPos.getY(), hubPos.getHeading() + 180))
                .splineTo(new Vector2d(duckSpinPos.getX(), duckSpinPos.getY()), duckSpinPos.getHeading(), velConstraints, accConstraint)
                .build();

        taskList.add(new SplineMoveTask(drive, traj2));
        taskList.add(new DuckCarouselSpinTask(robotHardware, startPosStr));

        Trajectory traj3 = drive.trajectoryBuilder(duckSpinPos, true)
                .splineTo(new Vector2d(p1.getX(), p1.getY()), p1.getHeading(), velConstraints, accConstraint)
                .splineTo(new Vector2d(parkPos.getX(), parkPos.getY()), parkPos.getHeading(), velConstraints, accConstraint)
                .build();

        taskList.add(new SplineMoveTask(drive, traj3));




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

        return taskList;
    }

    public ArrayList<RobotControl> prepareSHDelivery_WarehouseParkingWall_TaskList(){
        return null;
    }

    public void prepareSHDelivery_Carousel_StorageParkingTaskList(){

    }

    public void prepareSHDelivery_WarehousePickup_SHDelivery_WarehouseParkingWallTaskList(){

    }

    void init(){

        switch(driverOptions.getStartingPositionModes()){
            case("RED_LEFT"):
//                starting_x = 0;
//                starting_y = 0;
                red_left_starting_heading = 0;
                shipping_hub_x = red_shipping_hub_x;
                shipping_hub_y = red_shipping_hub_y;
//                hub_heading = red_left_angle_to_hub;
                direction = AngleMath.Direction.CLOCKWISE;
                break;
            case("RED_RIGHT"):
//                red_right_starting_x = 0; //212;
//                red_right_starting_y = 0; //20;
                shipping_hub_x = red_shipping_hub_x;
                shipping_hub_y = red_shipping_hub_y;
                hub_heading = red_right_angle_to_hub;
                direction = AngleMath.Direction.ANTI_CLOCKWISE;
                break;
            case("BLUE_LEFT"):
//                starting_x = 212;
//                starting_y = 365;
                shipping_hub_x = blue_shipping_hub_x;
                shipping_hub_y = blue_shipping_hub_y;
                hub_heading = blue_left_angle_to_hub;
                direction = AngleMath.Direction.CLOCKWISE;
                break;
            case("BLUE_RIGHT"):
                starting_x = 90;
                starting_y = 365;
                shipping_hub_x = blue_shipping_hub_x;
                shipping_hub_y = blue_shipping_hub_y;
                hub_heading = blue_right_angle_to_hub;
                direction = AngleMath.Direction.ANTI_CLOCKWISE;
                break;
            default:
        }
    }

}