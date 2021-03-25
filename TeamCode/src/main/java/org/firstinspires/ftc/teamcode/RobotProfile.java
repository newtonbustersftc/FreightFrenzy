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
    CVParam goalCvParam;
    CVParam wobbleCvParam;
    HardwareSpec hardwareSpec;
    FeedForwardParam rrFeedForwardParam;
    HashMap<String, AutoPose> poses;

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
        hardwareSpec.armDeliverPos = 1400;
        hardwareSpec.armGrabPos = 1900;
        hardwareSpec.armHoldPos = 450;
        hardwareSpec.armInitPos = 200;
        hardwareSpec.armPower = 0.5;
        hardwareSpec.armPowerLow = 0.3;
        hardwareSpec.armReverseDelay = 100;
        hardwareSpec.armReverseDelta = 30;
        hardwareSpec.grabberOpenPos = 0.21;
        hardwareSpec.grabberClosePos = 0.64;
        hardwareSpec.shooterOpen = 0.54;
        hardwareSpec.shooterClose = 0.67;
        hardwareSpec.shootVelocity = -1200;
        hardwareSpec.autonomousShootVelocity = -1220;
        hardwareSpec.shootVelocityStartAutonomous = -1240;
        hardwareSpec.shootBarVelocity = -1120;
        hardwareSpec.shootServoDelay = 800;
        hardwareSpec.shootDelay =300;
        hardwareSpec.intakePower = -1.0;
        hardwareSpec.ringHolderUp = 0.338;
        hardwareSpec.ringHolderDown = 0.555;
        hardwareSpec.ringHolderDown2 = 0.48;
        hardwareSpec.ringPusherUp = 0.78;
        hardwareSpec.ringPusherShoot = 0.5;
        hardwareSpec.ringPusherDown = 0.17;
        hardwareSpec.cameraForwardDisplacement = 1.5f;
        hardwareSpec.cameraVerticalDisplacement = 16.5f;
        hardwareSpec.cameraLeftDisplacement = -1.5f;
        hardwareSpec.cameraHeadingOffset = -2.5f;
        hardwareSpec.shootingAngle = Math.atan(8.0/70.0);
        hardwareSpec.shootingDistBase = 70;
        hardwareSpec.shootingVelocityInch = -75.0/36.0;
        hardwareSpec.wobbleOffsetX = 16; // inch
        hardwareSpec.wobbleOffsetY = 6.5; // inch

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

        goalCvParam = new CVParam();
        goalCvParam.cropTop = 0;
        goalCvParam.cropBottom = 70;
        goalCvParam.maskLowerH = 0;
        goalCvParam.maskLowerS = 0;
        goalCvParam.maskLowerV = 135;
        goalCvParam.maskUpperH = 255;
        goalCvParam.maskUpperS = 30;
        goalCvParam.maskUpperV = 255;
        goalCvParam.minArea = 8000;


        wobbleCvParam = new CVParam();
        wobbleCvParam.cropTop = 0;
        wobbleCvParam.cropBottom = 70;
        wobbleCvParam.maskLowerH = 230;
        wobbleCvParam.maskLowerS = 150;
        wobbleCvParam.maskLowerV = 100;
        wobbleCvParam.maskUpperH = 15;
        wobbleCvParam.maskUpperS = 255;
        wobbleCvParam.maskUpperV = 255;
        wobbleCvParam.minArea = 100;

        poses = new HashMap<String, AutoPose>();
        poses.put("START", new AutoPose(-66,-20,0));
        poses.put("TRANSIT", new AutoPose(-25, -20, 0));
        poses.put("TRANSIT2", new AutoPose(-25, -14, 180));
        poses.put("TRANSIT3", new AutoPose(-25, -48, 0));
        poses.put("SHOOT", new AutoPose(-5, -30, -12));
        poses.put("SHOOT-DRIVER", new AutoPose(-5, -42, -5));
        poses.put("SHOOT-PB-RIGHT", new AutoPose(-62, -62, 0));
        poses.put("SHOOT-PB-LEFT", new AutoPose(-62, 14.5, 0));
        poses.put("SHOOT-PB-RIGHT-1", new AutoPose(-62, -59, 0));
        poses.put("SHOOT-PB-LEFT-1", new AutoPose(-62, 11.5, 0));
        poses.put("SHOOT-POWER-BAR-1", new AutoPose(-5, -7, -5));
        poses.put("SHOOT-POWER-BAR-2", new AutoPose(-5, -14.5, -5));
        poses.put("SHOOT-POWER-BAR-3", new AutoPose(-5, -22, -5));
        poses.put("AUTOA-SHOOT-POWER-BAR-1", new AutoPose(-5, -5.5, -5));
        poses.put("AUTOA-SHOOT-POWER-BAR-2", new AutoPose(-5, -12.5, -5));
        poses.put("AUTOA-SHOOT-POWER-BAR-3", new AutoPose(-5, -20.5, -5));
        poses.put("AUTOB-SHOOT-POWER-BAR-1", new AutoPose(-40, -5.5, -5));
        poses.put("AUTOB-SHOOT-POWER-BAR-2", new AutoPose(-40, -12.5, -5));
        poses.put("AUTOB-SHOOT-POWER-BAR-3", new AutoPose(-40, -20.5, -5));

        poses.put("START_STATE", new AutoPose(-66,-33,0));
        poses.put("FIRST_SHOOT_STATE", new AutoPose(-38, -33, -5));
        poses.put("QUAD_COLLISION_STATE", new AutoPose(-30,-33,  -5));
        poses.put("RINGS_PICK1_STATE", new AutoPose(-26,-33,-5));
        poses.put("RINGS_PICK2_STATE", new AutoPose(-18,-33,-5));
        poses.put("C_1_STATE", new AutoPose(45,-53, -30));
        poses.put("WG2_PICKPre_STATE", new AutoPose(20,-48, 180));
        poses.put("WG2_PICK_STATE", new AutoPose(-32,-54,180));
        poses.put("C_WG2_DELIVERPre_STATE", new AutoPose(-25,-48, -5));
        poses.put("C_WG2_DELIVER_STATE", new AutoPose(39,-54, -48));
        poses.put("C_PARKING", new AutoPose(5,-34, -48));

        poses.put("A-1", new AutoPose(8, -46, -90));
        poses.put("A-WB2Pre", new AutoPose(-30, -42, -180));
        poses.put("A-WB2", new AutoPose(-30, -52, -180));
//        poses.put("A-WB2Pre", new AutoPose(-30, -42, -180));
//        poses.put("A-WB2", new AutoPose(-38, -42, -180));
        poses.put("A-2", new AutoPose(-2, -46, -90));

        poses.put("B-1", new AutoPose(19,-32,0));
        poses.put("B-PICK", new AutoPose(10, -40, -90));
        poses.put("B-WB2Pre", new AutoPose(-30,-25,-150));
        poses.put("B-WB2", new AutoPose(-28,-50,-180));
        //poses.put("B-WB2Pre", new AutoPose(-57,-28,-95));
        //poses.put("B-WB2", new AutoPose(-57,-34,-90));
        poses.put("B-2", new AutoPose(14,-34,0));

        poses.put("C-1", new AutoPose(37,-56,-45));
        poses.put("C-PICK", new AutoPose(-12, -35, -180));
        poses.put("C-PICK2", new AutoPose(-20, -35, -180));
        poses.put("C-PICK3Back", new AutoPose(-30, -35, -180));
        poses.put("C-WB2Pre", new AutoPose(-57,-28,-95));
        poses.put("C-WB2", new AutoPose(-57,-34,-90));
        poses.put("C-2", new AutoPose(40,-58,-15));
        poses.put("C2-PIC", new AutoPose(-32, -4, 270));

        poses.put("PARKING", new AutoPose(8, -30, 0));
        poses.put("AUTO-TRACKER-IMG", new AutoPose(36, -36, 0));

        poses.put("ADS-WOBBLE-PICK", new AutoPose(42, -40, -45));
        poses.put("ADS-WOBBLE-DROP1", new AutoPose(-58, -48, 180));
        poses.put("ADS-FINAL-1", new AutoPose(-50, -48, 90));

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
        double grabberOpenPos;
        double grabberClosePos;
        int armInitPos;
        int armGrabPos;
        int armHoldPos;
        int armDeliverPos;
        long armReverseDelay;
        int armReverseDelta;
        double armPower;
        double armPowerLow;
        double shooterOpen;
        double shooterClose;
        double ringPusherUp;
        double ringPusherShoot;
        double ringPusherDown;
        int shootVelocity;
        int autonomousShootVelocity;
        int shootVelocityStartAutonomous;
        int shootBarVelocity;
        long shootServoDelay;
        int shootDelay;
        double intakePower;
        double ringHolderUp;
        double ringHolderDown;
        double ringHolderDown2;
        float cameraForwardDisplacement;
        float cameraVerticalDisplacement;
        float cameraLeftDisplacement;
        double cameraHeadingOffset;
        double shootingAngle;
        double shootingDistBase;
        double shootingVelocityInch;
        double wobbleOffsetX;
        double wobbleOffsetY;
    }

    class FeedForwardParam {
        double kA;
        double kV;
        double kStatic;
    }

    class AutoPose{
        double x;
        double y;
        double heading;

        AutoPose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
}
