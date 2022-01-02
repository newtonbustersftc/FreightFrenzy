package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.opencv.core.Rect;
import org.firstinspires.ftc.teamcode.RobotProfile;

public class HubVisionMathModel {
    // When in drop off position

    RobotProfile profile;

    public HubVisionMathModel(RobotProfile profile){
        this.profile = profile;
    }
    // When it's 20 inches away from the drop off
    // pixels on the image of the pole
    // pixels from left
    // inches camera to pole

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
            double targetX = dist * (profile.hvParam.DIST_CENTER - profile.hvParam.FINAL_CENTER) / (profile.hvParam.DIST_INCH - profile.hvParam.FINAL_DIST) + profile.hvParam.FINAL_CENTER;
            return profile.hvParam.errorFactor * (lastResult.centerX - targetX) / dist;
        }
    }

    class LineCandidate {
        public double x;
        public double angle;
        public double length;
    };

    Result lastResult = new Result();

    LineCandidate[] candidates = new LineCandidate[profile.hvParam.MAX_LINE_CANDIDATES];
    int cnt = 0;
    Line centerLine;
    public HubVisionMathModel() {
        cnt = 0;
        centerLine = new Line(new Vector2D(0, profile.hvParam.CENTER_Y), new Vector2D(320, profile.hvParam.CENTER_Y),0);
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
        while (ndx<cnt && ndx<profile.hvParam.MAX_LINE_CANDIDATES) {
            if (Math.abs(candidates[ndx].x-lc.x)<=profile.hvParam.MIN_LINE_GAP) {
                // we found our candidate
                double newLength = lc.length+candidates[ndx].length;
                candidates[ndx].x = candidates[ndx].x * candidates[ndx].length/newLength + lc.x * lc.length/newLength;
                candidates[ndx].angle = candidates[ndx].angle * candidates[ndx].length/newLength + lc.angle * lc.length/newLength;
                candidates[ndx].length = newLength;
                return;
            }
            ndx++;
        }
        if (ndx<profile.hvParam.MAX_LINE_CANDIDATES) {
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
        double avg = (profile.hvParam.FINAL_DIST*profile.hvParam.FINAL_WIDTH + profile.hvParam.DIST_INCH*profile.hvParam.DIST_WIDTH)/2;
        return avg / lastResult.width;
    }
}
