package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * a Road Runner localizer that uses the Intel T265 Realsense
 */
@Config
public class RealSenseLocalizer implements Localizer {
    private static double INCH_TO_METER = 0.0254;

    private Pose2d t265Offset = new Pose2d(6.25, -1, 0);    // If Robot Center is 0,0,0, where is T265 and angle
    private Pose2d originOffset = new Pose2d(0,0,0);    // When setPostEstimate, remember where the robot center origin
    private static Pose2d postEstimate = new Pose2d();
    private T265Camera.CameraUpdate t265Update;
    private RobotHardware robotHardware;
    private RobotProfile robotProfile;
    public static T265Camera slamra;
    int sampleN;

    public RealSenseLocalizer(RobotHardware robotHardware, boolean resetPos, RobotProfile robotProfile) {
        this.robotProfile = robotProfile;
        this.robotHardware = robotHardware;
        // TODO initialize t265Offset from Profile
        postEstimate = new Pose2d();
        slamra = robotHardware.getT265Camera();
        sampleN = 0;
    }

    /**
     * @return
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        //The FTC265 library uses Ftclib geometry
        // Need to convert T265 post to robot pose, and then to again relative to origin pose
        if (t265Update != null) {
            // The way we oriented the sensor, Y is our field X, X is our field Y, Heading is robot heading
            Pose2d t265Pose = new Pose2d(t265Update.pose.getY()/INCH_TO_METER, t265Update.pose.getX()/INCH_TO_METER, t265Update.pose.getHeading());
            Pose2d centerPose = translateRobotCenter(t265Pose);
            postEstimate = fieldTranslate(originOffset, centerPose); //offsets the pose to be what the pose estimate is;
            sampleN++;
            if (sampleN%10==0) {
                Logger.logFile("getPoseEstimate - t265Pose:" + t265Pose + " center:" + centerPose + " origin: " + originOffset +
                        " estimate:" + postEstimate);
            }
        } else {
            RobotLog.v("NULL Camera Update");
        }
        return new Pose2d(postEstimate.getX(), -postEstimate.getY(), norm(postEstimate.getHeading()));
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        RobotLog.v("Set Pose to " + pose2d.toString());
        long startTime = System.currentTimeMillis();
        t265Update = slamra.getLastReceivedCameraUpdate();
        while (t265Update.confidence == T265Camera.PoseConfidence.Failed && (System.currentTimeMillis() - startTime) < 1000) {
            t265Update = slamra.getLastReceivedCameraUpdate();
        }
        if (t265Update.confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("Failed to setPoseEstimate, no t265Update");
            Logger.logFile("Failed to setPoseEstimate, no t265Update");
        }
        Pose2d t265Pose = new Pose2d(t265Update.pose.getY()/INCH_TO_METER, t265Update.pose.getX()/INCH_TO_METER, t265Update.pose.getHeading());
        Pose2d origin0 = translateRobotCenter(t265Pose);
        originOffset = new Pose2d(origin0.getX()-(pose2d.getX()*Math.cos(pose2d.getHeading())+pose2d.getY()*Math.sin(pose2d.getHeading())),
                origin0.getY()-(pose2d.getX()*Math.sin(pose2d.getHeading())-pose2d.getY()*Math.cos(pose2d.getHeading())), origin0.getHeading()-pose2d.getHeading());
        Logger.logFile("setPoseEstimate - t265Pose:" + t265Pose + " origin:" + originOffset);
        RobotLog.v("setPoseEstimate - t265Pose:" + t265Pose + " origin:" + originOffset);
    }

    public Pose2d translateRobotCenter(Pose2d t265Pose) {
        double deltaX = -Math.sin(t265Pose.getHeading())*t265Offset.getY() - Math.cos(t265Pose.getHeading())*t265Offset.getX();
        double deltaY = -Math.cos(t265Pose.getHeading())*t265Offset.getY() + Math.sin(t265Pose.getHeading())*t265Offset.getX();
        return new Pose2d(t265Pose.getX() + deltaX, t265Pose.getY() + deltaY, t265Pose.getHeading());
    }

    public Pose2d fieldTranslate(Pose2d orig, Pose2d robot) {
        double deltaX = (robot.getX() - orig.getX())*Math.cos(-orig.getHeading()) +
                (robot.getY() - orig.getY())*Math.sin(-orig.getHeading());
        double deltaY = (robot.getX() - orig.getX()) * Math.sin(orig.getHeading()) +
                (robot.getY() - orig.getY())*Math.cos(- orig.getHeading());
        return new Pose2d(deltaX, deltaY, robot.getHeading() - orig.getHeading());
    }

    public String getT265Confidence() {
        return t265Update.confidence.toString();
    }

    /**
     * updates the camera.  Used in
     * @see //org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense in update()
     */
    @Override
    public void update() {
        Thread.yield();
        t265Update = slamra.getLastReceivedCameraUpdate();
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
        ChassisSpeeds velocity = t265Update.velocity;

        double vX = (velocity.vyMetersPerSecond/INCH_TO_METER)*Math.cos(-originOffset.getHeading()) +
                (velocity.vxMetersPerSecond/INCH_TO_METER)*Math.sin(-originOffset.getHeading());
        double vY = (velocity.vyMetersPerSecond/INCH_TO_METER) * Math.sin(originOffset.getHeading()) +
                (velocity.vxMetersPerSecond/INCH_TO_METER)*Math.cos(- originOffset.getHeading());
        if (sampleN%100==0) {
            Logger.logFile("getPoseVelocity:" + vX + ", " + vY + ", " + Math.toDegrees(velocity.omegaRadiansPerSecond));
        }

        return new Pose2d(vX, -vY, velocity.omegaRadiansPerSecond);
    }

    /**
     * @param angle angle in radians
     * @return normiazled angle between ranges 0 to 2Pi
     */
    private double norm(double angle)
    {
        while (angle>Math.PI*2) angle-=Math.PI*2;
        while (angle<=0) angle+=Math.PI*2;
        return angle;
    }

}