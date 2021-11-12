package org.firstinspires.ftc.teamcode;


public class LiftBucketTask implements RobotControl {
        RobotHardware robotHardware;
        RobotProfile robotProfile;
        RobotHardware.LiftPosition liftPos;
        long startTime;

        public LiftBucketTask(RobotHardware hardware, RobotProfile profile, RobotHardware.LiftPosition liftPos) {
            this.robotHardware = hardware;
            this.robotProfile = profile;
            this.liftPos = liftPos;
        }

        public String toString() {
            return "Lift Bucket to " + liftPos;
        }

        @Override
        public void prepare() {
            robotHardware.setLiftPosition(liftPos);
            startTime = System.currentTimeMillis();
        }

        @Override
        public void execute() {
        }
        @Override
        public void cleanUp() {
        }

        // isDone only when the navigator x,y,h all within the range of pos1 and pos2
        @Override
        public boolean isDone() {
            return (System.currentTimeMillis()-startTime)>200 && !robotHardware.isLiftMoving();
        }
}
