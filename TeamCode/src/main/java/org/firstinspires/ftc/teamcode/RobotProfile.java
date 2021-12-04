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
        File file1 = new File("/sdcard/FIRST/profileA.json");
        if (!file1.exists()) {
            file1 = new File("/sdcard/FIRST/profileB.json");
        }
        Gson gson = new Gson();
        return gson.fromJson(new FileReader(file1), RobotProfile.class);
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
        hardwareSpec.AXEL_TRANS_PID_1 = 5;
        hardwareSpec.AXEL_TRANS_PID_2 = 0.1;
        hardwareSpec.AXEL_TRANS_PID_3 = 0;
        hardwareSpec.LATERIAL_TRANS_PID_1 = 5;
        hardwareSpec.LATERIAL_TRANS_PID_2 = 0.1;
        hardwareSpec.LATERIAL_TRANS_PID_3 = 0;
        hardwareSpec.leftRightWheelDist = 41;//cm left right dist, from tuning
        hardwareSpec.leftEncodeForwardSign = 1;
        hardwareSpec.rightEncoderForwardSign = -1;
        hardwareSpec.horizontalEncoderForwardSign = -1;
        hardwareSpec.intakeVelocity = 1000.0;
        hardwareSpec.intakeTime = 5000;
        hardwareSpec.duckDriveVelocity = 550;
        hardwareSpec.duckAutoVelocity = 200;
        hardwareSpec.duckSpinTime = 3000;
        hardwareSpec.cameraForwardDisplacement = 1.5f;
        hardwareSpec.cameraVerticalDisplacement = 16.5f;
        hardwareSpec.cameraLeftDisplacement = -1.5f;
        hardwareSpec.cameraHeadingOffset = -2.5f;
        hardwareSpec.realSenseAngleModifier = Math.PI/2;
        hardwareSpec.liftPositionZero = 20;
        hardwareSpec.liftPositionBottom = 540;
        hardwareSpec.liftPositionIntermediate = 900;
        hardwareSpec.liftPositionMiddle = 1200;
        hardwareSpec.liftPositionTop = 1950;
        hardwareSpec.liftMotorPower = 0.3;
        hardwareSpec.boxFlapOpen = 0.25;
        hardwareSpec.boxFlapClose = 0.35;
        hardwareSpec.dropToHubTime = 2000;

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
        poses.put("BLUE_DEPOT_START", new AutoPose(0,0,0));
        poses.put("RED_START_DUCK", new AutoPose(0,0,180));
        poses.put("BLUE_DEPOT_PREHUB", new AutoPose(0,-3,0));
        poses.put("BLUE_DEPOT_HUB", new AutoPose(-20,-27,80));
        poses.put("BLUE_DEPOT_PREWALL", new AutoPose(0,-1.85,0));
        poses.put("BLUE_DEPOT_PICKUPWALL", new AutoPose(38,-1.85,0));
        poses.put("BLUE_DEPOT_PARKWALL", new AutoPose(30,-1.85,0));
        poses.put("BLUE_DEPOT_PRECENTRAL", new AutoPose(0,-23,0));
        poses.put("BLUE_DEPOT_PARKCENTRAL", new AutoPose(35,-25,0));
        poses.put("RED_DEPOT_START", new AutoPose(0,0,0));
        poses.put("RED_DEPOT_PREHUB", new AutoPose(0,3,0));
        poses.put("RED_DEPOT_HUB", new AutoPose(-20.5,27,290));
        poses.put("RED_DEPOT_PREWALL", new AutoPose(0,1.85,0));
        poses.put("RED_DEPOT_PICKUPWALL", new AutoPose(38,1.85,0));
        poses.put("RED_DEPOT_PARKWALL", new AutoPose(30,1.85,0));
        poses.put("RED_DEPOT_PRECENTRAL", new AutoPose(0,23,0));
        poses.put("RED_DEPOT_PARKCENTRAL", new AutoPose(35,25,0));
        poses.put("RED_DUCK_START", new AutoPose(0,0,180.0));
        poses.put("RED_DUCK_PREHUB", new AutoPose(0,3,180.0));
        poses.put("RED_DUCK_HUB", new AutoPose(19,25,70.0));
        poses.put("RED_DUCK_CAROUSEL", new AutoPose(-19,7,235.0));
        poses.put("RED_DUCK_AFTERCAROUSEL", new AutoPose(-10.0,19.0,315.0));
        poses.put("RED_DUCK_PREWALL", new AutoPose(56,1.75,0));
        poses.put("RED_DUCK_PARKWALL", new AutoPose(75,1,0));
        poses.put("RED_DUCK_PRECENTRAL", new AutoPose(45,25,0));
        poses.put("RED_DUCK_PARKCENTRAL", new AutoPose(80,25,0));
        poses.put("RED_DUCK_PREPARKSTORAGE", new AutoPose(0,2,270.0));
        poses.put("RED_DUCK_PARKSTORAGE", new AutoPose(0,22,0));
        poses.put("BLUE_DUCK_START", new AutoPose(0,0,180.0));
        poses.put("BLUE_DUCK_PREHUB", new AutoPose(0,-3,180.0));
        poses.put("BLUE_DUCK_HUB", new AutoPose(19,-23.5,290.0));
        poses.put("BLUE_DUCK_CAROUSEL", new AutoPose(-24,-8,135.0));
        poses.put("BLUE_DUCK_AFTERCAROUSEL", new AutoPose(-8.0,-19.0,60.0));
        poses.put("BLUE_DUCK_PREWALL", new AutoPose(56,-1.75,0));
        poses.put("BLUE_DUCK_PARKWALL", new AutoPose(75,-1.5,0));
        poses.put("BLUE_DUCK_PRECENTRAL", new AutoPose(45,-25,0));
        poses.put("BLUE_DUCK_PARKCENTRAL", new AutoPose(80,-25,0));
        poses.put("BLUE_DUCK_PREPARKSTORAGE", new AutoPose(0,-2,270.0));
        poses.put("BLUE_DUCK_PARKSTORAGE", new AutoPose(0,-22,0));

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
        double AXEL_TRANS_PID_1;
        double AXEL_TRANS_PID_2;
        double AXEL_TRANS_PID_3;
        double LATERIAL_TRANS_PID_1;
        double LATERIAL_TRANS_PID_2;
        double LATERIAL_TRANS_PID_3;

        double leftRightWheelDist;
        int leftEncodeForwardSign;
        int rightEncoderForwardSign;
        int horizontalEncoderForwardSign;
        double intakeVelocity;
        int intakeTime;
        int duckDriveVelocity;
        int duckAutoVelocity;
        long duckSpinTime;
        float cameraForwardDisplacement;
        float cameraVerticalDisplacement;
        float cameraLeftDisplacement;
        double cameraHeadingOffset;
        double realSenseAngleModifier;
        int liftPositionZero;
        int liftPositionBottom;
        int liftPositionIntermediate;
        int liftPositionMiddle;
        int liftPositionTop;
        double liftMotorPower;
        double boxFlapOpen;
        double boxFlapClose;
        int dropToHubTime;
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


