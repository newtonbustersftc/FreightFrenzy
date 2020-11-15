package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.BulkMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.BulkTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class RobotHardware {
    ExpansionHubMotor rrMotor, rlMotor, frMotor, flMotor;
    ExpansionHubEx expansionHub1, expansionHub2;
    RevBulkData bulkData1, bulkData2;
    BulkMecanumDrive mecanumDrive;
    BulkTrackingWheelLocalizer trackingWheelLocalizer;
    RobotVision robotVision;
    //ExpansionHubServo ;

    //Servo ;
    //DigitalChannel ;
    //Rev2mDistanceSensor ;
    RobotProfile profile;
    boolean isPrototype = false;
    BNO055IMU imu1;

    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        this.profile = profile;
        try {
            if (hardwareMap.get("LiftMotor")!=null) {
                isPrototype = false;
            }
        }
        catch (IllegalArgumentException ex) {
            isPrototype = true;
        }
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
        imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
        imu1.initialize(parameters);

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        getRobotVision();
        if (!isPrototype) {
            expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        }
        Logger.logFile("Encoder Read:" + rrMotor.getCurrentPosition() + "," + rlMotor.getCurrentPosition());
        getBulkData1();
        getBulkData2();

        DriveConstants.kA = profile.rrFeedForwardParam.kA;
        DriveConstants.kV = profile.rrFeedForwardParam.kV;
        DriveConstants.kStatic = profile.rrFeedForwardParam.kStatic;
        SampleMecanumDrive.HEADING_PID = new PIDCoefficients(profile.rrHeadingPID.p,profile.rrHeadingPID.i,profile.rrHeadingPID.d);
        SampleMecanumDrive.TRANSLATIONAL_PID = new PIDCoefficients(profile.rrTranslationPID.p,profile.rrTranslationPID.i,profile.rrTranslationPID.d);
        mecanumDrive = new BulkMecanumDrive(this, rrMotor, rlMotor, frMotor, flMotor);
        trackingWheelLocalizer = new BulkTrackingWheelLocalizer(this);
        mecanumDrive.setLocalizer(trackingWheelLocalizer);

    }

    public BulkMecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public RobotVision getRobotVision(){ return robotVision;}

    public BulkTrackingWheelLocalizer getTrackingWheelLocalizer(){
        return trackingWheelLocalizer;
    }

    public void getBulkData1() {
        bulkData1 = expansionHub1.getBulkInputData();
        Logger.logFile(("getBulkData1: " + bulkData1.getMotorCurrentPosition(rrMotor) + "," + bulkData1.getMotorCurrentPosition(rlMotor)));
        Logger.logFile("Encoder Read:" + rrMotor.getCurrentPosition() + "," + rlMotor.getCurrentPosition());
    }

    public void getBulkData2() {
        if (!isPrototype) {
            bulkData2 = expansionHub2.getBulkInputData();
        }
        else {
            // so we have something
            bulkData2 = expansionHub1.getBulkInputData();
        }
    }

    public int getEncoderCounts(EncoderType encoder) {
        if(encoder == EncoderType.LEFT) {
            return profile.hardwareSpec.leftEncodeForwardSign * bulkData1.getMotorCurrentPosition(rlMotor);
        }
        else if(encoder == EncoderType.RIGHT) {
            return profile.hardwareSpec.rightEncoderForwardSign * bulkData1.getMotorCurrentPosition(rrMotor);
        }
        else if(encoder == EncoderType.HORIZONTAL) {
            return profile.hardwareSpec.horizontalEncoderForwardSign * bulkData1.getMotorCurrentPosition(frMotor);
        } else {
            return 0;
        }
    }

    public double getEncoderVelocity(EncoderType encoder) {
        if(encoder == EncoderType.LEFT) {
            return profile.hardwareSpec.leftEncodeForwardSign * bulkData1.getMotorVelocity(rlMotor);
        }
        else if(encoder == EncoderType.RIGHT) {
            return profile.hardwareSpec.rightEncoderForwardSign * bulkData1.getMotorVelocity(rrMotor);
        }
        else if(encoder == EncoderType.HORIZONTAL) {
            return profile.hardwareSpec.horizontalEncoderForwardSign * bulkData1.getMotorVelocity(frMotor);
        } else {
            return 0;
        }
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
        }else if(sign ==1){  //left side less encoder counts
            frontLeft = power *0.95 * Math.cos(angle) + rotation;
            frontRight = power * Math.sin(angle) - rotation;
            rearLeft = power * 0.95 * Math.sin(angle) + rotation;
            rearRight = power * Math.cos(angle) - rotation;
        }else if(sign == 2){   //right side less encoder counts
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

        setMotorPower(frontLeft, frontRight, rearLeft, rearRight);
    }

    public void mecanumDrive2(double power, double angle, double rotation){

        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        double robotAngle = Math.PI / 2 - angle - Math.PI / 4;
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

//    public void rotateGrabberOriginPos() {
//        clampRotationServo.setPosition(profile.hardwareSpec.clampAngleNormal);
//    }

    public float getGyroAngle() {
        float angle1 = -imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        //float angle2 = robotHardware.imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        //return (angle1 + angle2) / 2;
        return angle1;
    }

    /**
     * WARNING WARNING WARNING ****
     * Sensor distance call not supported by BulkData read, this call takes 33ms to complete
     * do not call as part of the loop constantly.
     * @return
     */

    public enum EncoderType {LEFT, RIGHT, HORIZONTAL}

}
