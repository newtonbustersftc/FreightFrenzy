package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.HubVisionMathModel;

public class AutoHubApproachTask implements RobotControl {
    double power;
    double[] prevWidth;  // a ring buffer of previous distance
    int prevNdx;
    RobotHardware robot;
    RobotProfile profile;
    RobotVision robotVision;

    transient boolean completed;
    transient int loopCount = 0;
    //transient double pathAngle = 0;
    transient double pathDistance = 0;
    transient double targetAngle;
    transient double lastToTargetDist;
    transient double minPower = 0.2;

    public AutoHubApproachTask(RobotHardware robot, RobotProfile profile){
        this.robot = robot;
        this.profile = profile;
        this.robotVision = robot.getRobotVision();
        completed = false;
        power = 0.3;    // default
        prevWidth = new double[5];  // previous 10 width
        for(int i = 0; i< prevWidth.length; i++) {
            prevWidth[i] = -9999;
        }
        prevNdx = 0;
        loopCount = 0;
    }

    public String toString() {
        return "AutoHubApproach Task";
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setMinPower(double minPower){
        this.minPower = minPower;
    }

    /**
     * Multiple criteria to determine completion
     * 1. 0.5cm from target
     * 2. Overshoot (target dist go further from previous measure)
     * 3. Travel distance greater than planned distance
     * 4. No movement for last 40 measures
     * @return
     */
    public boolean isDone(){
        HubVisionMathModel.Result r = robotVision.getLastHubResult();
        if (r==null) {
            return true;
        }

        if (r.width> HubVisionMathModel.FINAL_WIDTH-5) {
            return true;
        }
        return false;
    }

    public void prepare(){
        HubVisionMathModel.Result r = robotVision.getLastHubResult();
    }

    public void execute() {
        HubVisionMathModel.Result r = robotVision.getLastHubResult();
        if (r!=null) {
            double pwr = (power - minPower) * (HubVisionMathModel.FINAL_WIDTH - r.width) / HubVisionMathModel.FINAL_WIDTH + minPower;
            Logger.logFile("AutoHub " + r + " pwr:" + pwr);
            robot.mecanumDrive2(pwr, Math.PI, r.getAngleCorrection());
        }
    }

    public void cleanUp(){
        robot.setMotorPower(0,0,0,0);
    }
}

