package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing without changing the corresponding
 * slot's motor direction
 */
public class BulkEncoder {
    private final static int CPS_STEP = 0x10000;

    private static double inverseOverflow(double input, double estimate) {
        double real = input;
        while (Math.abs(estimate - real) > CPS_STEP / 2.0) {
            real += Math.signum(estimate - real) * CPS_STEP;
        }
        return real;
    }

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private RobotHardware robotHardware;
    private RobotHardware.EncoderType encoderType;
    private NanoClock clock;

    private Direction direction;

    private int lastPosition;
    private double velocityEstimate;
    private double lastUpdateTime;

    public BulkEncoder(RobotHardware robotHardware, RobotHardware.EncoderType encoderType, NanoClock clock) {
        this.robotHardware = robotHardware;
        this.encoderType = encoderType;
        this.clock = clock;

        this.direction = Direction.FORWARD;

        this.lastPosition = 0;
        this.velocityEstimate = 0.0;
        this.lastUpdateTime = clock.seconds();
    }

    public BulkEncoder(RobotHardware robotHardware, RobotHardware.EncoderType encoderType) {
        this(robotHardware, encoderType, NanoClock.system());
    }

    public Direction getDirection() {
        return direction;
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getCurrentPosition() {
        int multiplier = direction.getMultiplier();
        int currentPosition = robotHardware.getEncoderCounts(encoderType) * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            velocityEstimate = (currentPosition - lastPosition) / dt;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    public double getRawVelocity() {
        int multiplier = direction.getMultiplier();
        return robotHardware.getEncoderVelocity(encoderType) * multiplier;
    }

    public double getCorrectedVelocity() {
        return inverseOverflow(getRawVelocity(), velocityEstimate);
    }
}
