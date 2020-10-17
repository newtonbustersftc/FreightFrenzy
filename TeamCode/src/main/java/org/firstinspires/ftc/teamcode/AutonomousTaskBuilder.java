package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class AutonomousTaskBuilder {
    int delay;
    RobotProfile robotProfile;
    RobotHardware robotHardware;
    RobotNavigator navigator;
    DriverOptions driverOptions;
    ArrayList<RobotControl> taskList = new ArrayList<>();
    RobotPosition lastPos;
    PIDMecanumMoveTask lastMovement;

    public AutonomousTaskBuilder(DriverOptions driverOptions, int skyStonePosition, RobotHardware robotHardware, RobotNavigator navigator, RobotProfile robotProfile) {
        this.delay = driverOptions.getDelay();
        this.driverOptions = driverOptions;
        this.robotHardware = robotHardware;
        this.navigator = navigator;
        this.robotProfile = robotProfile;
    }

    void addMovement(RobotPosition beginP, RobotPosition endP) {
        lastMovement = new PIDMecanumMoveTask(robotHardware, robotProfile, navigator);
        lastMovement.setPath(beginP, endP);
        taskList.add(lastMovement);
        lastPos = endP;
    }


    void populateCommonTask(){
    }
}