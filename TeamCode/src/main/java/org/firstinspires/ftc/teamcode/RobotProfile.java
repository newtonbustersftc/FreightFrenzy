package org.firstinspires.ftc.teamcode;

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
    PIDParam rrHeadingPID;
    PIDParam rrTranslationPID;
    CVParam cvParam;
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
        rrFeedForwardParam.kV = 0.01588;
        rrFeedForwardParam.kStatic = 0.04386;

        hardwareSpec = new HardwareSpec();
        hardwareSpec.trackWheelDiameter = 3.8;   //cm diameter
        hardwareSpec.trackWheelCPR = 4000;
        hardwareSpec.leftRightWheelDist = 41;//cm left right dist, from tuning
        hardwareSpec.leftEncodeForwardSign = 1;
        hardwareSpec.rightEncoderForwardSign = -1;
        hardwareSpec.horizontalEncoderForwardSign = -1;
        hardwareSpec.armDeliverPos = -800;
        hardwareSpec.armGrabPos = -1700;
        hardwareSpec.armHoldPos = -650;
        hardwareSpec.armInitPos = 0;
        hardwareSpec.armPower = 0.3;
        hardwareSpec.grabberOpenPos = 0.21;
        hardwareSpec.grabberClosePos = 0.64;
        hardwareSpec.shooterOpen = 0.15;
        hardwareSpec.shooterClose = 0.65;
        hardwareSpec.shootPower = -0.50;
        hardwareSpec.shootPowerBar = -0.45;
        hardwareSpec.shootServoDelay = 250;
        hardwareSpec.shootDelay = 2000;


        cvParam = new CVParam();
        cvParam.cropTop = 20;
        cvParam.maskLowerH = 20;
        cvParam.maskLowerS = 150;
        cvParam.maskLowerV = 100;
        cvParam.maskUpperH = 30;
        cvParam.maskUpperS = 255;
        cvParam.maskUpperV = 255;
        cvParam.minArea = 5;

        poses = new HashMap<String, AutoPose>();
        poses.put("START", new AutoPose(-66,-18,0));
        poses.put("TRANSIT", new AutoPose(-25, -18, 0));
        poses.put("TRANSIT2", new AutoPose(-25, -14, 180));
        poses.put("TRANSIT3", new AutoPose(-25, -48, 0));
        poses.put("SHOOT", new AutoPose(-5, -28, -10));
        poses.put("SHOOT-POWER-BAR-1", new AutoPose(-5, -5, -7));
        poses.put("SHOOT-POWER-BAR-2", new AutoPose(-5, -5, -10));
        poses.put("SHOOT-POWER-BAR-3", new AutoPose(-5, -5, -13));

        poses.put("A-1", new AutoPose(8, -40, -90));
        poses.put("A-WB2Pre", new AutoPose(-30, -41, -180));
        poses.put("A-WB2", new AutoPose(-38, -41, -180));
        poses.put("A-2", new AutoPose(-10, -50, -30));

        poses.put("B-1", new AutoPose(18,-36,0));
        poses.put("B-WB2Pre", new AutoPose(-57,-28,-95));
        poses.put("B-WB2", new AutoPose(-57,-34,-90));
        poses.put("B-2", new AutoPose(12,-44,0));

        poses.put("C-1", new AutoPose(48,-51,-45));
        poses.put("C-2", new AutoPose(40,-58,-15));


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
    }

    class CVParam {
        int maskUpperH;
        int maskUpperS;
        int maskUpperV;
        int maskLowerH;
        int maskLowerS;
        int maskLowerV;
        int cropTop;
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
        double armPower;
        double shooterOpen;
        double shooterClose;
        double shootPower;
        double shootPowerBar;
        long shootServoDelay;
        int shootDelay;
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
