package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

/**
 * Utility to calculate robot distance and heading, even pose based on
 * the recognized target paper position on the image
 *
 * Distance
 * f = d * Z / D
 * which
 * <LI>f - focal length</LI>
 * <LI>d - screen size pixels</LI>
 * <LI>Z - distance</LI>
 * <LI>D - actual size</LI>
 */
public class GoalTargetRecognition {
    public static final double INCH_TO_CM = 2.54;
    public static final double FIELD_WIDTH = 144 * 2.54;
    public static final double FOCAL_LENGTH_VERTICAL = 850;
    public static final double FOCAL_LENGTH_HORIZONTAL = 820;
    public static final double VIEWPORT_WIDTH = 640;
    public static final double VIEWPORT_HEIGHT = 480;
    public static final double PROFILE_TARGET_WIDTH= 117;       //px
    public static final double PROFILE_TARGET_HEIGHT = 68;      //px
    public static final double PROFILE_TARGET_DIST = 181 + 17;  //cm
    public static final double PROFILE_CENTER_X = 297;
    public static final double TARGET_WIDTH = 11 * INCH_TO_CM;
    public static final double TARGET_HEIGHT = 8.5 * INCH_TO_CM;
    public static final double TARGET_POS_X = FIELD_WIDTH/2;
    public static final double TARGET_POS_Y = -FIELD_WIDTH/4;
    public static final double CAMERA_INSET = 17;   //cm
    public static final double ROBOT_WIDTH = 18 * INCH_TO_CM;
    public static double ANGLE_OFFSET;

    static {
        // calculate ANGLE_OFFSET
        double offsetToCenter = (VIEWPORT_WIDTH/2-PROFILE_CENTER_X) * TARGET_WIDTH / PROFILE_TARGET_WIDTH;
        ANGLE_OFFSET = Math.atan(offsetToCenter / PROFILE_TARGET_DIST);
    }

    Vector2D[] targetPoints;
    double distInch;
    double targetAngle;

    public GoalTargetRecognition(Vector2D[] targetPoints) {
        ArrayList<Vector2D> tmpList = new ArrayList<Vector2D>();
        Collections.addAll(tmpList, targetPoints);
        tmpList.sort(new CornerComparator());
        this.targetPoints = tmpList.toArray(new Vector2D[0]);
    }

    public String toString() {
        return "Dist:" + getDistanceInch() + " Angle:" + Math.toDegrees(getTargetAngle());
    }

    /**
     * use the picture middle height and focal length to calculate the distance
     * @return distance in cm from center of robot to the target
     */
    public double getDistanceInch() {
        double d = (targetPoints[2].getY() + targetPoints[3].getY() - targetPoints[0].getY() - targetPoints[1].getY())/2;
        return ((FOCAL_LENGTH_VERTICAL * TARGET_HEIGHT / d) - CAMERA_INSET + ROBOT_WIDTH/2)/INCH_TO_CM;
    }

    /**
     * use the center of the image offset to the picture center to estimate the current robot heading offset to goal
     * @return Angle to center in Radian, to the
     */
    public double getTargetAngle() {
        double c = (targetPoints[0].getX() + targetPoints[1].getX() + targetPoints[2].getX() + targetPoints[3].getX())/4;
        double xoffsetPx = VIEWPORT_WIDTH / 2 - c;
        // d is distance from picture center to camera
        double tmp = (targetPoints[2].getY() + targetPoints[3].getY() - targetPoints[0].getY() - targetPoints[1].getY())/2;
        double distCM = (FOCAL_LENGTH_VERTICAL * TARGET_HEIGHT / tmp);
        double xoffsetCM = xoffsetPx * distCM / FOCAL_LENGTH_HORIZONTAL;
        double angle = Math.atan(xoffsetCM / distCM) - ANGLE_OFFSET;
        return angle;
    }

    public boolean isValid() {
        // if any point is close to the left/right edge, then no good
        for(int i=0; i<4; i++) {
            if (targetPoints[i].getX() < VIEWPORT_WIDTH*0.05 || targetPoints[i].getX() > VIEWPORT_WIDTH*0.95) {
                return false;
            }
        }
        // if the width/height ratio is > 11/8.5 or <0.8
        double width2=targetPoints[1].getX() + targetPoints[3].getX() - targetPoints[0].getX() - targetPoints[2].getX();
        double height2=targetPoints[2].getY() + targetPoints[3].getY() - targetPoints[0].getY() - targetPoints[1].getY();
        if (width2/height2 > 1.3 || width2/height2<0.8) {
            return false;
        }
        // if the diagonal line is not close to equal
        double d1 = targetPoints[0].distance(targetPoints[3]);
        double d2 = targetPoints[1].distance(targetPoints[2]);
        if (d1/d2 > 1.10 || d1/d2<0.9) {
            return false;
        }
        return true;
    }

    /**
     * Use the left/right height, and the top/bottom width to estimate the robot position, and heading
     *
     * TO DO
     */

}

/**
 * After soring should have the corners order by top left, top right, bottom left, bottom right order
 */
class CornerComparator implements Comparator<Vector2D> {

    public int compare(Vector2D v1, Vector2D v2) {
        return (int)Math.signum(v1.getX() * 0.2 + v1.getY() - v2.getX() * 0.2 - v2.getY());
    }
}