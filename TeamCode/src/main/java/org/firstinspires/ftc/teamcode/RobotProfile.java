package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.HashMap;

public class RobotProfile {

    PIDParam headingPID;
    PIDParam distancePID;
    PIDParam shootPID;
    PIDParam rrHeadingPID;
    PIDParam rrTranslationPID;
    CVParam cvParam;
    HardwareSpec hardwareSpec;
    FeedForwardParam rrFeedForwardParam;
    HashMap<String, AutoPose> poses;
    Movement movement;

    public static RobotProfile loadFromFile(File file) throws FileNotFoundException {
        Gson gson = new Gson();
        return gson.fromJson(new FileReader(file), RobotProfile.class);
    }

    public void populateInitValue() {
        headingPID = new PIDParam();
        headingPID.p = 5;
        headingPID.i = 0.05;
        headingPID.d = 0.0;
        distancePID = new PIDParam();
        distancePID.p = 0.5;
        distancePID.i = 0.01;
        distancePID.d = 0.0;
        shootPID = new PIDParam();
        shootPID.p = 24; //original: 10
        shootPID.i = 0.3;   //original: 0.3
        shootPID.d = 0.0;
        shootPID.f = 14;  //original: 14
        rrHeadingPID = new PIDParam();
        rrHeadingPID.p = 8;
        rrHeadingPID.i = 0.05;
        rrHeadingPID.d = 0;
        rrTranslationPID = new PIDParam();
        rrTranslationPID.p = 10;
        rrTranslationPID.i = 0.1;
        rrTranslationPID.d = 0;
        rrFeedForwardParam = new FeedForwardParam();
        rrFeedForwardParam.kA = 0.00001;
        rrFeedForwardParam.kV = 0.01578;
        rrFeedForwardParam.kStatic = 0.06141;

        hardwareSpec = new HardwareSpec();
        hardwareSpec.trackWheelDiameter = 3.8;   //cm diameter
        hardwareSpec.trackWheelCPR = 4000;
        hardwareSpec.leftRightWheelDist = 41;//cm left right dist, from tuning
        hardwareSpec.leftEncodeForwardSign = 1;
        hardwareSpec.rightEncoderForwardSign = -1;
        hardwareSpec.horizontalEncoderForwardSign = -1;
        hardwareSpec.intakeVelocity = 1000.0;
        hardwareSpec.duckVelocity = 550;
        hardwareSpec.duckSpinTime = 5000;
        hardwareSpec.cameraForwardDisplacement = 1.5f;
        hardwareSpec.cameraVerticalDisplacement = 16.5f;
        hardwareSpec.cameraLeftDisplacement = -1.5f;
        hardwareSpec.cameraHeadingOffset = -2.5f;
        hardwareSpec.realSenseAngleModifier = Math.PI/2;
        hardwareSpec.liftPositionZero = 0;
        hardwareSpec.liftPositionBottom = 540;
        hardwareSpec.liftPositionMiddle = 1200;
        hardwareSpec.liftPositionTop = 1625;
        hardwareSpec.liftMotorPower = 0.3;
        hardwareSpec.boxFlapOpen = 0.25;
        hardwareSpec.boxFlapClose = 0.35;

        cvParam = new CVParam();
        cvParam.cropTop = 20;
        cvParam.cropBottom = 0;
        cvParam.maskLowerH = 20;
        cvParam.maskLowerS = 150;
        cvParam.maskLowerV = 100;
        cvParam.maskUpperH = 30;
        cvParam.maskUpperS = 255;
        cvParam.maskUpperV = 255;
        cvParam.minArea = 5;

        poses = new HashMap<String, AutoPose>();
        poses.put("RED_START_LEFT", new AutoPose(0,0,180));
        poses.put("RED_LEFT_1", new AutoPose(-10, -10, 135));
        poses.put("RED_CAROUSEL", new AutoPose(-20, -2, 210));
        poses.put("RED_LEFT_2", new AutoPose(-15, -40, 180));
        poses.put("RED_LEFT_HUB", new AutoPose(6, -40, 180));

        poses.put("RED_START_RIGHT", new AutoPose(0,0,180));
        poses.put("RED_RIGHT_1", new AutoPose(-10, -10, 135));
        poses.put("RED_CAROUSEL", new AutoPose(-20, -2, 210));
        poses.put("RED_RIGHT_2", new AutoPose(-15, -40, 180));
        poses.put("RED_RIGHT_HUB", new AutoPose(6, -40, 180));

        poses.put("BLUE_START_RIGHT", new AutoPose(0, 0, 180));
        poses.put("BLUE_RIGHT_1", new AutoPose(-10, 10, 225));
        poses.put("BLUE_CAROUSEL", new AutoPose(-20, 2, 135));
        poses.put("BLUE_RIGHT_2", new AutoPose(-15, 40, 180));
        poses.put("BLUE_RIGHT_HUB", new AutoPose(6, 40, 180));

        poses.put("BLUE_START_LEFT", new AutoPose(0, 0, 180));
        poses.put("BLUE_LEFT_1", new AutoPose(-10, 10, 225));
        poses.put("BLUE_CAROUSEL", new AutoPose(-20, 2, 135));
        poses.put("BLUE_LEFT_2", new AutoPose(-15, 40, 180));
        poses.put("BLUE_LEFT_HUB", new AutoPose(6, 40, 180));

        movement = new Movement();
        movement.forwardStopDist = 25;
        movement.strifeStopDist = 17;
        movement.rotateStopAngle = .1;
    }

    public Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    public void saveToFile(File file) throws FileNotFoundException {
        PrintWriter out = new PrintWriter(file);
        GsonBuilder builder = new GsonBuilder();
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String json = gson.toJson(this);
        out.println(json);
        out.close();
    }

    class PIDParam {
        double p;
        double i;
        double d;
        double f;
    }

    class CVParam {
        int maskUpperH;
        int maskUpperS;
        int maskUpperV;
        int maskLowerH;
        int maskLowerS;
        int maskLowerV;
        int cropTop;
        int cropBottom;
        int minArea;
    }

    class HardwareSpec {
        double trackWheelDiameter;   //cm diameter
        double trackWheelCPR;
        double leftRightWheelDist;
        int leftEncodeForwardSign;
        int rightEncoderForwardSign;
        int horizontalEncoderForwardSign;
        double intakeVelocity;
        int duckVelocity;
        long duckSpinTime;
        float cameraForwardDisplacement;
        float cameraVerticalDisplacement;
        float cameraLeftDisplacement;
        double cameraHeadingOffset;
        double realSenseAngleModifier;
        int liftPositionZero;
        int liftPositionBottom;
        int liftPositionMiddle;
        int liftPositionTop;
        double liftMotorPower;
        double boxFlapOpen;
        double boxFlapClose;
    }

    class FeedForwardParam {
        double kA;
        double kV;
        double kStatic;
    }

    class AutoPose {
        double x;
        double y;
        double heading;

        AutoPose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
    class Movement {
        double strifeStopDist;
        double forwardStopDist;
        double rotateStopAngle;
        }
}


