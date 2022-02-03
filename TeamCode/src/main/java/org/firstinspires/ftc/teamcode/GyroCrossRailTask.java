package org.firstinspires.ftc.teamcode;

/**
 * Use Gyro sensor to cross the rail while maintaining heading no change.
 * cross with higher power, then use lower power
 * Use cumulative min encoder increment to precise cross the rail
 * Will disable T265 confidence check during the move
 */
public class GyroCrossRailTask implements RobotControl {
    double powerHigh, powerLow;

    RobotHardware robot;
    RobotProfile profile;
    long crossCnt;
    long finalCnt;
    double startHeading;
    int totalCnt;
    int frCnt, flCnt, rrCnt, rlCnt;

    /**
     * Constructor
     * @param robot
     * @param profile
     * @param crossCnt  encoder count to cross the rail using high power
     * @param finalCnt  final encoder count to stop
     */
    public GyroCrossRailTask(RobotHardware robot, RobotProfile profile, long crossCnt, long finalCnt) {
        this.robot = robot;
        this.profile = profile;
        this.crossCnt = crossCnt;
        this.finalCnt = finalCnt;
        powerHigh = 0.75;
        powerLow = 0.15;
    }

    public String toString() {
        return "GyroCrossRailTask";
    }

    public void prepare() {
        startHeading = robot.getImuHeading();
        totalCnt = 0;
        robot.setIgnoreT265Confidence(true);
        frCnt = robot.getEncoderCounts(RobotHardware.EncoderType.FRONT_RIGHT);
        flCnt = robot.getEncoderCounts(RobotHardware.EncoderType.FRONT_LEFT);
        rrCnt = robot.getEncoderCounts(RobotHardware.EncoderType.REAR_RIGHT);
        rlCnt = robot.getEncoderCounts(RobotHardware.EncoderType.REAR_LEFT);
        Logger.logFile("GyroCrossRailTask prepare heading: " + Math.toDegrees(startHeading));
    }

    public boolean isDone() {
        // at least one of the wheels should not be slipping :)
        int frCnt1 = robot.getEncoderCounts(RobotHardware.EncoderType.FRONT_RIGHT);
        int flCnt1 = robot.getEncoderCounts(RobotHardware.EncoderType.FRONT_LEFT);
        int rrCnt1 = robot.getEncoderCounts(RobotHardware.EncoderType.REAR_RIGHT);
        int rlCnt1 = robot.getEncoderCounts(RobotHardware.EncoderType.REAR_LEFT);
        totalCnt = totalCnt + Math.min(Math.min(frCnt1-frCnt, flCnt1-flCnt), Math.min(rrCnt1-rrCnt, rlCnt1-rlCnt));
        frCnt = frCnt1;
        flCnt = flCnt1;
        rrCnt = rrCnt1;
        rlCnt = rlCnt1;
        return totalCnt > finalCnt;
    }

    public void execute() {
        double currHeading = robot.getImuHeading();
        double corr = (currHeading - startHeading);
        double baseP;
        if (totalCnt<crossCnt) {
            baseP = powerHigh;
        }
        else {
            baseP = powerLow;
            corr = corr/3;
        }
        robot.setMotorPower(baseP + corr, baseP - corr, baseP+corr, baseP-corr);
    }

    public void cleanUp() {
        Logger.logFile("GyroCrossRailTask cnt increment " + totalCnt);
        robot.setMotorPower(0, 0, 0, 0);
        robot.setIgnoreT265Confidence(false);
    }
}

