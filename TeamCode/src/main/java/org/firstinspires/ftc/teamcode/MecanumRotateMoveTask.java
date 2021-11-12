package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import org.firstinspires.ftc.teamcode.util.AngleMath;

/**
 * 2019.12.01
 * Created by Athena Z.
 * ----
 * Now includes move to new position too.  It won't be done until the distance to target is within 1 cm
 * and the angle is within 1 degree (1/180 * PI)
 */

public class MecanumRotateMoveTask implements RobotControl {
    Pose2d startPose, endPose;
    double power = 0.5;

    AngleMath.Direction turnDirection;
    double turnSign;
    RobotHardware robot;
    RobotProfile profile;
    Localizer localizer;
    double currentAngle;
    double targetAngle;
    double minPower = 0.2;
    long timeOut = 60000;
    long startTime;
    boolean slowRotate;
    static double rotateStop = Math.PI*30/180;
    static double minDeltaAngle = Math.PI/180;

    public MecanumRotateMoveTask(RobotHardware robot, RobotProfile profile) {
        this.robot = robot;
        this.profile = profile;
        this.localizer = robot.getLocalizer();
        startPose = null;
        endPose = null;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setMinPower(double minPower){
        this.minPower = minPower;
    }

    public void setRotateHeading(Pose2d startPose, Pose2d endPose, AngleMath.Direction turnDirection) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.turnDirection = turnDirection;
    }

    public String toString() {
        return "Rotate from " + startPose + " to " + endPose + " curr:";
    }

    public void setTimeOut(long timeOut) {
        this.timeOut = timeOut;
    }

    public boolean isDone() {
        Pose2d currPose = localizer.getPoseEstimate();
        double dist = Math.hypot(endPose.getY() - currPose.getY(), endPose.getX() - currPose.getX());
        //12/12 try to loose the condition so hook off easier
        if ((System.currentTimeMillis() - startTime) > timeOut) {
            return true;
        }
        if ((dist < 0.5) && (AngleMath.absDeltaAngle(currPose.getHeading(), endPose.getHeading()) < minDeltaAngle)){
            Logger.logFile("Done Rotation: " + currPose);
            return true;
        } else {
            return false;
        }
    }


    public void prepare() {
        robot.setMotorStopBrake(true);
        currentAngle = startPose.getHeading();
        targetAngle = endPose.getHeading();
        turnSign = (turnDirection==AngleMath.Direction.ANTI_CLOCKWISE)?-1:1;
        if (turnDirection==AngleMath.Direction.STRAIGHT) {
            slowRotate = true;
        }
        else {
            slowRotate = false;
        }
        startTime = System.currentTimeMillis();
        Logger.logFile("Prepare Rotation - start: " + startPose + ", end: " + endPose);
    }

    public void execute() {
        /**
         * RobotProfile.java    class Movement  double rotateStopAngle;
         * movement.rotateStopAngle = 5;
         */
        Pose2d currPose = localizer.getPoseEstimate();
        currentAngle = currPose.getHeading();
        // calculate the target angle based on current position and end position
        // and move the robot to the target position
        double moveAngle = -Math.atan2(endPose.getY() - currPose.getY(), endPose.getX() - currPose.getX());
        double dist = Math.hypot(endPose.getY() - currPose.getY(), endPose.getX() - currPose.getX());
        double movePwr, rotatePwr;
        if (dist>15) {
            movePwr = power;
        }
        else {
            movePwr = minPower + dist/15 * (power - minPower);
        }
        // now need to determine which way to turn
        // when it's greater than 45 degree, we use the direction prescribed by start/end angle (rightTurnSign)
        if (!slowRotate && (AngleMath.absDeltaAngle(currentAngle, endPose.getHeading(), turnDirection)>rotateStop)) {
            rotatePwr = turnSign * power;
        }
        else {
            slowRotate = true;
            // now direction should be get close to finish, if overshot, turn back
            rotatePwr = minPower + AngleMath.absDeltaAngle(currentAngle, endPose.getHeading())/rotateStop * (power - minPower);
            if (currentAngle < endPose.getHeading() || (currentAngle-endPose.getHeading()>Math.PI)) {
                // we going to turn left
                rotatePwr = -rotatePwr;
            }
        }
        robot.mecanumDrive2(movePwr, moveAngle - currentAngle, rotatePwr);
        Logger.logFile("Rotating current " + currPose +
                " move angle:" + Math.toDegrees((moveAngle)) + " Pwr:" + movePwr + " rot:" + rotatePwr);
    }

    public void cleanUp() {
        robot.setMotorPower(0,0,0,0);
    }
}