package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.MecanumRotateMoveTask;

public class AngleMath {
    public enum Direction { STRAIGHT, CLOCKWISE, ANTI_CLOCKWISE }

    public static double absDeltaAngle(double ang1, double ang2) {
        ang1 = normalizeAngle(ang1);
        ang2 = normalizeAngle(ang2);
        if (Math.abs(ang1 - ang2) > Math.PI){
            return Math.PI * 2 - Math.abs(ang1 - ang2);
        }
        return Math.abs(ang1-ang2);
    }

    public static double absDeltaAngle(double fromAngle, double toAngle, Direction rotationDir) {
        fromAngle = normalizeAngle(fromAngle);
        toAngle = normalizeAngle(toAngle);
        double ang;
        if (rotationDir== Direction.ANTI_CLOCKWISE) {
            if (toAngle>fromAngle) {
                ang = toAngle - fromAngle;
            }
            else {
                ang = fromAngle + Math.PI*2 - toAngle;
            }
        }
        else {  // CLOCKWISE
            if (fromAngle>toAngle) {
                ang = fromAngle - toAngle;
            }
            else {
                ang = fromAngle + Math.PI*2 - toAngle;
            }
        }
        return ang;
    }

    /**
     * Normalize the angle to make sure it's between 0 .. PI*2 range
     * @param ang
     * @return
     */
    public static double normalizeAngle(double ang) {
        while (ang>Math.PI*2) {
            ang = ang - Math.PI*2;
        }
        while (ang<0) {
            ang = ang + Math.PI*2;
        }
        return ang;
    }

}
