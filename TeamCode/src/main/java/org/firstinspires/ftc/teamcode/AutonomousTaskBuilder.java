package org.firstinspires.ftc.teamcode;

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
    RobotNavigator navigator;
    PIDMecanumMoveTask lastMovement;
    RobotPosition lastPos;
    int delay;
    String startingPositionModes;
    String parking;
    String duckPosition;
    String deliverRoute;
    DriverOptions driverOptions;

    public AutonomousTaskBuilder(DriverOptions driverOptions, RobotHardware robotHardware, RobotProfile robotProfile, RobotVision.AutonomousGoal goal) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.delay = driverOptions.getDelay();
        this.startingPositionModes = driverOptions.getStartingPositionModes();
        this.parking = driverOptions.getParking();
        this.deliverRoute = driverOptions.getDeliveryRoutes();
        this.duckPosition = driverOptions.getDuckPosition();
        this.navigator = navigator;
        this.robotProfile = robotProfile;
    }

    void goWobbleTask(DriverOptions driverOptions){
        switch(driverOptions.getStartingPositionModes()){
            case("BLUE_LEFT"):
            case("BLUE_RIGHT"):
            case("RED_LEFT"):
            case("RED_RIGHT"):
        }
    }

    void addMovement(RobotPosition beginP, RobotPosition endP) {
        lastMovement = new PIDMecanumMoveTask(robotHardware, robotProfile);
        lastMovement.setPath(beginP, endP);
        taskList.add(lastMovement);
        lastPos = endP;
    }
}