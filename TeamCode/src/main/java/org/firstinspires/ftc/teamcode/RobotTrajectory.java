package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import java.util.ArrayList;

    public class RobotTrajectory {

        private Pose2d shootingPose;

        public ArrayList<Trajectory> createTrajectory() {
            Pose2d startPose = new Pose2d(-35, -47, Math.toRadians(280));
            shootingPose = new Pose2d(55, -47, Math.toRadians(0));
            double shootingHeading = 0;
            DriveConstraints constraints =

                    new DriveConstraints(60.0, 60.0, 0.0, Math.toRadians(270.0),  Math.toRadians(270.0), 0.0);
            TrajectoryBuilder builder = new TrajectoryBuilder(startPose, reverseOrForward(startPose), constraints);
            builder.splineToSplineHeading(shootingPose, shootingHeading);
            ArrayList<Trajectory> list = new ArrayList<Trajectory>();
            list.add(builder.build());
            return list;
        }


        public boolean reverseOrForward(Pose2d currPose) {

            if((Math.toRadians(90.0) <= currPose.getHeading()) || (currPose.getHeading() <= Math.toRadians(270.0))){
                if(currPose.getX() <= shootingPose.getX()){
                    return true;
                }else{
                    return false;
                }
            }
            if((Math.toRadians(0.0) <= currPose.getHeading()) || (currPose.getHeading() <= Math.toRadians(290.0))){
                if(currPose.getX() <= shootingPose.getX()){
                    return false;
                }else{
                    return true;
                }
            }
            if((Math.toRadians(270.0) <= currPose.getHeading()) || (currPose.getHeading() < Math.toRadians(360.0))){
                if(currPose.getX() <= shootingPose.getX()){
                    return false;
                }else{
                    return true;
                }
            }
            return true;
        }




}
