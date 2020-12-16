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
        hardwareSpec.armDeliverPos = -900;
        hardwareSpec.armGrabPos = -1890;
        hardwareSpec.armHoldPos = -600;
        hardwareSpec.armInitPos = 0;
        hardwareSpec.armPower = 0.3;
        hardwareSpec.grabberOpenPos = 0.21;
        hardwareSpec.grabberClosePos = 0.64;

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
        poses.put("TRANSIT2", new AutoPose(-25, -18, 180));
        poses.put("TRANSIT3", new AutoPose(-25, -52, 0));
        poses.put("SHOOT", new AutoPose(-5, -36, 0));
        poses.put("A-1", new AutoPose(8, -40, -90));
        poses.put("WB-2Pre", new AutoPose(-30, -40, -180));
        poses.put("WB-2", new AutoPose(-38, -40, -180));
        poses.put("A-2", new AutoPose(-10, -50, -30));

        poses.put("B-1", new AutoPose(20,-36,0));
        poses.put("B-WB2Pre", new AutoPose(-56,-28,-105));
        poses.put("B-WB2", new AutoPose(-56,-36,-90));
        poses.put("B-2", new AutoPose(16,-42,0));
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
