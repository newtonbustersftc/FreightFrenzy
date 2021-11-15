package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import com.spartronics4915.lib.T265Camera;
import java.text.DecimalFormat;

public class RobotHardware {
    public enum LiftPosition {
        ZERO, BOTTOM, MIDDLE, TOP;
        private static LiftPosition[] vals = values();

        public LiftPosition next() {
            return (this.ordinal() < vals.length - 1) ? vals[this.ordinal() + 1] : this;
        }

        public LiftPosition prev() { // can not goto init position
            return (this.ordinal() > 0) ? vals[this.ordinal() - 1] : vals[0];
        }
    }
    LiftPosition currLiftPos;

    HardwareMap hardwareMap;
    ExpansionHubMotor rrMotor, rlMotor, frMotor, flMotor;
    ExpansionHubMotor liftMotor, duckMotor, intakeMotor;
    DigitalChannel led4, led5;
    ExpansionHubServo boxFlapServo;
    DigitalChannel liftBottom;
    ExpansionHubEx expansionHub1, expansionHub2;
    RevBulkData bulkData1, bulkData2;
    SampleMecanumDrive mecanumDrive;
    RealSenseLocalizer realSenseLocalizer;
    RobotVision robotVision;
    static T265Camera t265 = null;
    DecimalFormat nf2 = new DecimalFormat("#.##");
    //Servo ;
    //DigitalChannel ;
    //Rev2mDistanceSensor ;
    RobotProfile profile;

    public int originLiftMotorEncoder;
    boolean isDelivered;

    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        Logger.logFile("RobotHardware init()");
        this.hardwareMap = hardwareMap;
        this.profile = profile;
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        rrMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("RRMotor");
        rlMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("RLMotor");
        frMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("FRMotor");
        flMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("FLMotor");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rlMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub");
        intakeMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("LiftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("DuckMotor");
        duckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //led4 = hardwareMap.digitalChannel.get("LED4");
        //led5 = hardwareMap.digitalChannel.get("LED5");
        //initLeds();

        //touch sensor
        liftBottom = hardwareMap.get(DigitalChannel.class, "LiftBottom");
        liftBottom.setMode(DigitalChannel.Mode.INPUT);

        boxFlapServo = (ExpansionHubServo)hardwareMap.servo.get("BoxFlap");

        Logger.logFile("Encoder Read:" + rrMotor.getCurrentPosition() + "," + rlMotor.getCurrentPosition());
        getBulkData1();
        getBulkData2();
//TODO
//        DriveConstants.kA = profile.rrFeedForwardParam.kA;
//        DriveConstants.kV = profile.rrFeedForwardParam.kV;
//        DriveConstants.kStatic = profile.rrFeedForwardParam.kStatic;
//        SampleMecanumDrive.HEADING_PID = new PIDCoefficients(profile.rrHeadingPID.p,profile.rrHeadingPID.i,profile.rrHeadingPID.d);
//        SampleMecanumDrive.TRANSLATIONAL_PID = new PIDCoefficients(profile.rrTranslationPID.p,profile.rrTranslationPID.i,profile.rrTranslationPID.d);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        if (t265 == null) {
            t265 = new T265Camera(new Transform2d(new Translation2d(0, 0), new Rotation2d(0)), 0, hardwareMap.appContext);
        }
        if (!t265.isStarted()) {
            t265.start();
        }
        realSenseLocalizer = new RealSenseLocalizer(this, true, profile);
        mecanumDrive.setLocalizer(realSenseLocalizer);
        robotVision = new RobotVision();

        currLiftPos = LiftPosition.ZERO;
    }

    public T265Camera getT265Camera() {
        return t265;
    }

    /**
     * We need this because Vuforia end of OpMode run because losing camera context
     */
    public void initRobotVision() {
        robotVision.init(hardwareMap, this, profile);
    }

    public RobotVision getRobotVision() {
        return robotVision;
    }

    public void initLeds() {
//        led1.setMode(DigitalChannel.Mode.OUTPUT);
//        led2.setMode(DigitalChannel.Mode.OUTPUT);
//        led3.setMode(DigitalChannel.Mode.OUTPUT);
//        led1.setState(true);
//        led2.setState(true);
//        led3.setState(true);
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public Localizer getLocalizer(){
        return realSenseLocalizer;
    }

    public void getBulkData1() {
        bulkData1 = expansionHub1.getBulkInputData();
    }

    public void getBulkData2() {
        bulkData2 = expansionHub2.getBulkInputData();
    }

    public int getEncoderCounts(EncoderType encoder) {
        if(encoder == EncoderType.LIFT) {
            return liftMotor.getCurrentPosition();
        }
        return 0;
    }

    public int getEncoderVelocity(EncoderType encoder) {
        return 0;   // TODO
    }

    public void mecanumDriveTest(double power, double angle, double rotation, int sign){
        double frontLeft = 0;
        double frontRight = 0;
        double rearLeft = 0;
        double rearRight = 0;
        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        //double robotAngle = Math.PI / 2 - angle - Math.PI / 4;
        if(sign == 0) {
            frontLeft = power * Math.cos(angle) + rotation;
            frontRight = power * Math.sin(angle) - rotation;
            rearLeft = power * Math.sin(angle) + rotation;
            rearRight = power * Math.cos(angle) - rotation;
        }
        else if(sign ==1){  //left side less encoder counts
            frontLeft = power *0.95 * Math.cos(angle) + rotation;
            frontRight = power * Math.sin(angle) - rotation;
            rearLeft = power * 0.95 * Math.sin(angle) + rotation;
            rearRight = power * Math.cos(angle) - rotation;
        }
        else if(sign == 2){   //right side less encoder counts
            frontLeft = power * Math.cos(angle) + rotation;
            frontRight = power * 0.95 * Math.sin(angle) - rotation;
            rearLeft = power * Math.sin(angle) + rotation;
            rearRight = power * 0.95 * Math.cos(angle) - rotation;
        }

        double biggest = 0.1;
        if (Math.abs(frontRight) > biggest){
            biggest = Math.abs(frontRight);
        }
        if (Math.abs(rearLeft) > biggest){
            biggest = Math.abs(rearLeft);
        }
        if (Math.abs(rearRight) > biggest){
            biggest = Math.abs(rearRight);
        }
        if (Math.abs(frontLeft) > biggest){
            biggest = Math.abs(frontLeft);
        }

        power = (power == 0 && rotation !=0) ? 1 : power;
        frontLeft = frontLeft/biggest*power;
        frontRight = frontRight/biggest*power;
        rearLeft = rearLeft/biggest*power;
        rearRight = rearRight/biggest*power;
//        Logger.logFile("Power - FL" + nf2.format(frontLeft) + " FR:"+ nf2.format(frontRight) +
//                        " RL:" + nf2.format(rearLeft) + " RR:" + nf2.format(rearRight));
        setMotorPower(frontLeft, frontRight, rearLeft, rearRight);
    }

    public void mecanumDrive2(double power, double angle, double rotation){

        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        double robotAngle = Math.PI / 2 + angle - Math.PI / 4;
        double frontLeft = power * Math.cos(robotAngle) + rotation;
        double frontRight = power * Math.sin(robotAngle) - rotation;
        double rearLeft = power * Math.sin(robotAngle) + rotation;
        double rearRight = power * Math.cos(robotAngle) - rotation;


        double biggest = 0;
        if (Math.abs(frontRight) > biggest){
            biggest = Math.abs(frontRight);
        }
        if (Math.abs(rearLeft) > biggest){
            biggest = Math.abs(rearLeft);
        }
        if (Math.abs(rearRight) > biggest){
            biggest = Math.abs(rearRight);
        }
        if (Math.abs(frontLeft) > biggest){
            biggest = Math.abs(frontLeft);
        }

        power = Math.max(power, Math.abs(rotation));
        frontLeft = frontLeft/biggest*power;
        frontRight = frontRight/biggest*power;
        rearLeft = rearLeft/biggest*power;
        rearRight = rearRight/biggest*power;

        setMotorPower(frontLeft, frontRight, rearLeft, rearRight);
    }

    public void setMotorPower(double flPower, double frPower, double rlPower, double rrPower) {
        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        rlMotor.setPower(rlPower);
        rrMotor.setPower(rrPower);
    }

    public void setMotorStopBrake(boolean brake) {
        flMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        frMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rlMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rrMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void startIntake() {
        intakeMotor.setVelocity(profile.hardwareSpec.intakeVelocity);
    }

    public void reverseIntake() {
        intakeMotor.setVelocity(-profile.hardwareSpec.intakeVelocity);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void startDuckAuto(int alliance) {
        if(alliance==1)  //red
            duckMotor.setVelocity(-alliance * profile.hardwareSpec.duckAutoVelocity);
        else
            duckMotor.setVelocity(-alliance * profile.hardwareSpec.duckAutoVelocity *0.9);
    }

    public void startDuck(int alliance){
        if(alliance==1)  //red
            duckMotor.setVelocity(-alliance * profile.hardwareSpec.duckDriveVelocity);
        else
            duckMotor.setVelocity(-alliance * profile.hardwareSpec.duckDriveVelocity *0.9);
    }

    public void stopDuck() {
        duckMotor.setPower(0);
    }

    public void setLiftMotorPosition(int pos) {
        liftMotor.setPower(profile.hardwareSpec.liftMotorPower);
        liftMotor.setTargetPosition(pos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLiftPosition(LiftPosition pos) {
        Logger.logFile("Setting lift position to: " + pos);
        currLiftPos = pos;
        liftMotor.setPower(profile.hardwareSpec.liftMotorPower);
        int newPosNum = 0;
        switch (pos) {
            case ZERO:
                newPosNum = profile.hardwareSpec.liftPositionZero;
                break;
            case BOTTOM:
                newPosNum = profile.hardwareSpec.liftPositionBottom;
                break;
            case MIDDLE:
                newPosNum = profile.hardwareSpec.liftPositionMiddle;
                break;
            case TOP:
                newPosNum = profile.hardwareSpec.liftPositionTop;
                break;
        }
        liftMotor.setTargetPosition(newPosNum);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void liftUp() {
        setLiftPosition(currLiftPos.next());
    }

    public void liftDown() {
        setLiftPosition(currLiftPos.prev());
    }

    public boolean isLiftMoving() {
        Logger.logFile("Lift moving velocity: " + liftMotor.getVelocity());
        return Math.abs(liftMotor.getVelocity())>10;
    }

    void resetLiftEncoderCount(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    /**
     * WARNING WARNING WARNING ****
     * Sensor distance call not supported by BulkData read, this call takes 33ms to complete
     * do not call as part of the loop constantly.
     * @return
     */

    public void stopAll() {
        setMotorPower(0, 0, 0, 0);
        liftMotor.setPower(0);
        intakeMotor.setPower(0);
        duckMotor.setPower(0);
        if (t265.isStarted()) {
            t265.stop();
        }
    }

    public enum EncoderType {LIFT}

    public void setLed4(boolean on) {
        led4.setState(!on);
    }

    public void setLed5(boolean on) {
        led5.setState(!on);
    }

    public void openBoxFlap(){
        if (currLiftPos!=LiftPosition.ZERO) {
            boxFlapServo.setPosition(0.3);
            //boxFlapServo.setPosition(profile.hardwareSpec.boxFlapOpen);
        }
    }

    public void closeBoxFlap(){
        boxFlapServo.setPosition(profile.hardwareSpec.boxFlapClose);
    }

    public LiftPosition getCurrLiftPos() {
        return currLiftPos;
    }

    public boolean liftBottomTouched() {
        // digital channel: low - touched, high - not touch
        return !liftBottom.getState();
    }
}
