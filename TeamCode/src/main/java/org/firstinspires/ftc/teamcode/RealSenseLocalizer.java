package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * a Road Runner localizer that uses the Intel T265 Realsense
 */
@Config
public class RealSenseLocalizer implements Localizer {

    private Pose2d poseOffset = new Pose2d();
    private static Pose2d mPoseEstimate = new Pose2d();
    private Pose2d rawPose = new Pose2d();
    private T265Camera.CameraUpdate up;
    private RobotHardware robotHardware;
    private RobotProfile robotProfile;

    public static T265Camera slamra;

    public static boolean makeCameraCenter = true;

    private static T265Camera.PoseConfidence poseConfidence;

    public RealSenseLocalizer(HardwareMap hardwareMap) {
        if (slamra==null) {
            slamra = new T265Camera(new Transform2d(new Translation2d(0, 0), new Rotation2d(0)), 0, hardwareMap.appContext);
        }
        else {
            slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)));
        }
        if (!slamra.isStarted()) {
            slamra.start();
        }
    }

    public RealSenseLocalizer(RobotHardware robotHardware, boolean resetPos, RobotProfile robotProfile) {
        this.robotProfile = robotProfile;
        this.robotHardware = robotHardware;
        poseOffset = new Pose2d();
        mPoseEstimate = new Pose2d();
        rawPose = new Pose2d();

        slamra = robotHardware.getT265Camera();
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)));
    }

    /**
     * @return
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        //variable up is updated in update()

        //The FTC265 library uses Ftclib geometry, so I need to convert that to road runner Geometry
        //TODO: convert all Ftclib geometry to ACME robotics geometry in T265Camera.java
        if (up != null) {
            Translation2d oldPose = up.pose.getTranslation();
            Rotation2d oldRot = up.pose.getRotation();
            //The T265's unit of measurement is meters.  dividing it by .0254 converts meters to inches.
            rawPose = new Pose2d(oldPose.getY() / .0254, oldPose.getX() / .0254, norm(oldRot.getRadians())); //raw pos
            mPoseEstimate = rawPose.plus(poseOffset); //offsets the pose to be what the pose estimate is;
        } else {
            RobotLog.v("NULL Camera Update");
        }
        return mPoseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        RobotLog.v("Set Pose to " + pose2d.toString());
        long startTime = System.currentTimeMillis();
        T265Camera.CameraUpdate update = slamra.getLastReceivedCameraUpdate();
        while (update.confidence== T265Camera.PoseConfidence.Failed && (System.currentTimeMillis() - startTime)<2000) {
            update = slamra.getLastReceivedCameraUpdate();
        }
        if (update.confidence== T265Camera.PoseConfidence.Failed) {
            RobotLog.e("setPoseEstimate didn't get camera update");
        }
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getY()* .0254, pose2d.getX() * 0.0254, new Rotation2d(pose2d.getHeading())));
        slamra.getLastReceivedCameraUpdate();
    }

    /**
     * updates the camera.  Used in
     * @see //org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense in update()
     */
    @Override
    public void update() {
        up = slamra.getLastReceivedCameraUpdate();
        poseConfidence = up.confidence;
    }

    /**
     No idea what the purpose getPoseVelocity.  Everything works fine by just using getPoseEstimate()
     That said, the code to get the velocity is comment out below.  Haven't testing it much
     and I don't know how well getting the velocity work or if use the velocity has any effect
     at all.
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        //variable up is updated in update()

        ChassisSpeeds velocity = up.velocity;
        return new Pose2d(velocity.vyMetersPerSecond /.0254,velocity.vxMetersPerSecond /.0254,velocity.omegaRadiansPerSecond);
    }

    /**
     * @param angle angle in radians
     * @return normiazled angle between ranges 0 to 2Pi
     */
    private double norm(double angle)
    {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }

//    /**
//     * DO NOT USE THiS
//     */
//    @Deprecated
//    @SuppressWarnings("SpellCheckingInspection")
//    private Pose2d adjustPosbyCameraPos()
//    {
//        double dist = Math.hypot(slamraX,slamraY); //distance camera is from center
//        double angle = Math.atan2(slamraY,slamraX);
//        double cameraAngle = mPoseEstimate.getHeading() - angle;
//        double detlaX = dist * Math.cos(cameraAngle);
//        double detlaY = dist * Math.sin(cameraAngle);
//        return mPoseEstimate.minus(new Pose2d(detlaX,detlaY));
//    }
}