package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.opencv.core.Rect;

public class HubVisionMathModel {
    // When in drop off position
    public static int FINAL_WIDTH = 58; // pixels on image of the pole
    public static int FINAL_CENTER = 80;   // pixels from left
    public static double FINAL_DIST = 11.0; // inch camera to pole
    // When it's 20 inches away from the drop off
    public static int DIST_WIDTH = 20;     // pixels on the image of the pole
    public static int DIST_CENTER= 170;     // pixels from left
    public static int DIST_INCH = 30;       // inches camera to pole
    public static int MIN_LINE_GAP = 5;
    public static int MAX_LINE_CANDIDATES = 4;
    public static int CENTER_Y = 70;
    public static double errorFactor = 0.1;

    public enum RecognitionResult { NONE, BOTH, SINGLE }

    public class Result {
        public RecognitionResult result;
        public double centerX;
        public double width;
        public double angle;
        public double distance;
        public double offset;
        public double leftX;
        public double rightX;

        public Result() {
            result = RecognitionResult.NONE;
        }
        public String toString() {
            if (result==RecognitionResult.NONE || result==RecognitionResult.SINGLE) {
                return result.toString() + " " + cnt;
            }
            else {
                return "HubRec W: " + width + " X:" + centerX + " Ac:" + getAngleCorrection();
            }
        }

        /**
         * Use the width to figure out the approximate distance, together with X to figure out the amount of correction needed
         * @return
         */
        public double getAngleCorrection() {
            double dist = calculateDistance();
            // at this distance, where is the correct X on picture?
            double targetX = dist * (DIST_CENTER - FINAL_CENTER) / (DIST_INCH - FINAL_DIST) + FINAL_CENTER;
            return errorFactor * (lastResult.centerX - targetX) / dist;
        }
    }

    class LineCandidate {
        public double x;
        public double angle;
        public double length;
    };

    Result lastResult = new Result();

    LineCandidate[] candidates = new LineCandidate[MAX_LINE_CANDIDATES];
    int cnt = 0;
    Line centerLine;
    public HubVisionMathModel() {
        cnt = 0;
        centerLine = new Line(new Vector2D(0, CENTER_Y), new Vector2D(320, CENTER_Y),0);
        candidates[0] = new LineCandidate();
        candidates[1] = new LineCandidate();
    }

    public void addRect(Rect rect) {
        cnt = 2;
        candidates[0].x = rect.x;
        candidates[1].x = rect.x + rect.width;
        candidates[0].angle = Math.PI/2;
        candidates[1].angle = Math.PI/2;
    }

    public void addLine(int x0, int y0, int x1, int y1) {
        if (x1!=x0 && Math.abs((y1-y0)/(x1-x0))<6) {
            return; // do simple math and reject lines not vertical enough
        }
        Line l = new Line(new Vector2D(x0, y0), new Vector2D(x1, y1), 0);
        LineCandidate lc = new LineCandidate();
        lc.x = l.intersection(centerLine).getX();
        lc.angle = Math.abs(Math.atan2(y1-y0, x1-x0));
        lc.length = Math.hypot(y1-y0, x1-x0);
        // add these lines to the array
        int ndx = 0;
        while (ndx<cnt && ndx<MAX_LINE_CANDIDATES) {
            if (Math.abs(candidates[ndx].x-lc.x)<=MIN_LINE_GAP) {
                // we found our candidate
                double newLength = lc.length+candidates[ndx].length;
                candidates[ndx].x = candidates[ndx].x * candidates[ndx].length/newLength + lc.x * lc.length/newLength;
                candidates[ndx].angle = candidates[ndx].angle * candidates[ndx].length/newLength + lc.angle * lc.length/newLength;
                candidates[ndx].length = newLength;
                return;
            }
            ndx++;
        }
        if (ndx<MAX_LINE_CANDIDATES) {
            candidates[ndx] = lc;
            cnt++;
        }
    }

    public Result getResult() {
        lastResult = new Result();
        if (cnt==2) {
            lastResult.result = RecognitionResult.BOTH;
            lastResult.centerX = (candidates[0].x + candidates[1].x)/2;
            lastResult.width = Math.abs(candidates[0].x - candidates[1].x) * Math.sin((candidates[0].angle + candidates[1].angle)/2);
        }
        else if (cnt==1) {
            lastResult.result = RecognitionResult.SINGLE;
        }
        else if (cnt==0) {
            lastResult.result = RecognitionResult.NONE;
        }
        else {
            lastResult.result = RecognitionResult.NONE;
        }
        return lastResult;
    }

    /** Take the average distance to width ratio of the final and 30 inch away, and use the current width to estimate
     * the distance
     * @return distance in inches
     */
    public double calculateDistance() {
        double avg = (FINAL_DIST*FINAL_WIDTH + DIST_INCH*DIST_WIDTH)/2;
        return avg / lastResult.width;
    }
}
