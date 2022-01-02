package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

/**
     * 2019.10.26
     * PID
     * Created by Claire Zheng
     */

public class PIDMecanumMoveTask implements RobotControl {

    Pose2d startPos, endPos;
    double offsetX;
    double offsetY;
    double power;
    double[] prevDist;  // a ring buffer of previous distance
    int prevNdx;

    transient RobotHardware robot;
    transient RobotProfile profile;
    transient Localizer navigator;
    transient PIDController pidPosition;
    transient PIDController pidHeading;
    transient boolean completed;
    transient int loopCount = 0;
    //transient double pathAngle = 0;
    transient double pathDistance = 0;
    transient double targetAngle;
    transient double lastToTargetDist;
    transient double minPower = 0.2;

    public PIDMecanumMoveTask(RobotHardware robot, RobotProfile profile) {
        this.robot = robot;
        this.profile = profile;
        this.navigator = robot.getLocalizer();
        startPos = null;
        endPos = null;
        completed = false;
        power = 0.5;    // default
        prevDist = new double[40];  // previous 10 distance
        for (int i = 0; i < prevDist.length; i++) {
            prevDist[i] = -9999;
        }
        prevNdx = 0;
        loopCount = 0;
    }

    public String toString() {
        return "Move " + startPos.getX() + "," + startPos.getY() + " -> " + endPos.getX() + "," + endPos.getY() + " curr:" + navigator.getPoseEstimate();
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    public void setPath(Pose2d startPos, Pose2d endPos) {
        this.startPos = startPos;
        this.endPos = endPos;
        Logger.logFile("Path Task:" + startPos + " TO " + endPos);
    }

    public void setRelativePath(double offsetX, double offsetY) {
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        startPos = null;
        endPos = null;
    }

    void setupPID() {
        pidPosition = new PIDController(profile.distancePID.p, profile.distancePID.i, profile.distancePID.d);
        pidHeading = new PIDController(profile.headingPID.p, profile.headingPID.i, profile.headingPID.d);
        pidPosition.reset();
        pidPosition.setSetpoint(0);
        pidPosition.setInputRange(-5, 5);       // off by 5 cm max
        pidPosition.setOutputRange(0, Math.PI / 20);  // this is the desired angle to move
        pidPosition.enable();
        pidHeading.reset();
        pidHeading.setInputRange(-Math.PI / 30, Math.PI / 30);  // of by 6 degrees max
        pidHeading.setOutputRange(0, .3);
        pidHeading.setSetpoint(0);
        pidHeading.enable();
    }

    public double getPosError() {
        Pose2d currentPose = navigator.getPoseEstimate();
        double currAngle = -Math.atan2(currentPose.getY() - startPos.getY(), currentPose.getX() - startPos.getX());
        double error = Math.hypot(currentPose.getY() - startPos.getY(), currentPose.getX() - startPos.getX()) * Math.sin(targetAngle - currAngle);
        return error;
    }

    public double getHeadingError() {
        Pose2d currentPose = navigator.getPoseEstimate();
        double headingErr = currentPose.getHeading() - endPos.getHeading();
        if (headingErr > Math.PI) {
            headingErr = headingErr - Math.PI * 2;
        } else if (headingErr < -Math.PI) {
            headingErr = headingErr + Math.PI * 2;
        }
        return headingErr;
    }

    /**
     * Multiple criteria to determine completion
     * 1. 0.5cm from target
     * 2. Overshoot (target dist go further from previous measure)
     * 3. Travel distance greater than planned distance
     * 4. No movement for last 40 measures
     *
     * @return
     */
    public boolean isDone() {
        Pose2d currentPose = navigator.getPoseEstimate();
        double toTargetDist = Math.hypot(endPos.getY() - currentPose.getY(), endPos.getX() - currentPose.getX());
        if (toTargetDist < 0.5) {
            return true;
        }
        if ((toTargetDist > lastToTargetDist + 0.5) && (lastToTargetDist < 5)) {
            Logger.logFile("Overshoot - toTarget:" + toTargetDist + " prev:" + lastToTargetDist);
            return true;
        } else {
            lastToTargetDist = toTargetDist;
        }
        double targetTravelDistance = Math.hypot(endPos.getY() - startPos.getY(), endPos.getX() - startPos.getX());
        double currentTravelDistance = Math.hypot(currentPose.getY() - startPos.getY(), currentPose.getX() - startPos.getX());
//        Logger.logFile("currentDistance: " + currentDistance + ", targetDistance: " + targetDistance);
        boolean noMovement = false;
        if (Math.abs(currentTravelDistance - prevDist[prevNdx]) < 1) {
            Logger.logFile("MoveBlocked - " + currentTravelDistance + " : " + prevDist[prevNdx]);
            noMovement = true;
        }
        prevDist[prevNdx] = currentTravelDistance;
        prevNdx = (prevNdx + 1) % prevDist.length;
//        Logger.logFile("navigator.getWorldY()=" + navigator.getWorldY() + " navigator.getWorldX() " + navigator.getWorldX());
//        Logger.logFile("noMovement = " + noMovement);
        return (currentTravelDistance > targetTravelDistance) || noMovement;
    }

    public void prepare() {
        Pose2d currentPose = navigator.getPoseEstimate();
        if (startPos == null && endPos == null) {
            startPos = new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
            endPos = new Pose2d(offsetX + currentPose.getX(), offsetY + currentPose.getY(), currentPose.getHeading());
        }
        lastToTargetDist = 10000;
        targetAngle = -Math.atan2(endPos.getY() - startPos.getY(), endPos.getX() - startPos.getX());        // CHANGED SIGN
        setupPID();
        Logger.logFile("PIDMecan - start:" + startPos + " end:" + endPos + " targetAngle" + Math.toDegrees(targetAngle));
    }

    public void execute() {
        Pose2d currentPose = navigator.getPoseEstimate();
        double posCorrection = pidPosition.performPID(getPosError());
        double headCorrection = pidHeading.performPID(getHeadingError());
        double targetDistance = Math.hypot(endPos.getY() - startPos.getY(), endPos.getX() - startPos.getX());
        double currentDistance = Math.hypot(currentPose.getY() - startPos.getY(), currentPose.getX() - startPos.getX());
        double pwr = power;
        if (targetDistance - currentDistance <= 15) {
            pwr = minPower + (targetDistance - currentDistance) / 15 * (power - minPower);
        }
        robot.mecanumDrive2(pwr, targetAngle - posCorrection - endPos.getHeading(), -headCorrection);
        loopCount++;
        if (loopCount % 10 == 0) {
            Logger.logFile("Pose:" + currentPose + " pwr:" + pwr + " postCorr:" + posCorrection + " headCorr:" + headCorrection);
        }
    }

    public void cleanUp() {
        robot.setMotorPower(0, 0, 0, 0);
    }
}

