package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.opencv.core.Rect;

public class HubVisionMathModel {
    // When in drop off position

    RobotProfile profile;

    public HubVisionMathModel(RobotProfile profile){
        this.profile = profile;
        cnt = 0;
        candidates = new LineCandidate[profile.hvParam.maxLineCandidates];
        centerLine = new Line(new Vector2D(0, profile.hvParam.centerY), new Vector2D(320, profile.hvParam.centerY),0);
        candidates[0] = new LineCandidate();
        candidates[1] = new LineCandidate();
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
            double targetX = dist * (profile.hvParam.awayCenter - profile.hvParam.finalCenter) / (profile.hvParam.awayDist - profile.hvParam.finalDist) + profile.hvParam.finalCenter;
            return profile.hvParam.errorFactor * (lastResult.centerX - targetX) / dist;
        }
    }

    class LineCandidate {
        public double x;
        public double angle;
        public double length;
    };

    Result lastResult = new Result();

    LineCandidate[] candidates;
    int cnt = 0;
    Line centerLine;
    Rect currRect = null;

    public void addRect(Rect rect) {
        if (currRect!=null) {
            // if two rec are close horizontally, and gap in between, and no overlap, then
            // probably caused by reflection, then combine
            if (Math.abs(rect.y + rect.height/2 - currRect.y - currRect.height/2)<10 &&
                    ((rect.x+rect.width) < currRect.x || (currRect.x + currRect.width) < rect.x)) {
                // Combine the existing currRect with rect
                if  ((rect.x+rect.width) < currRect.x) {    // new rect left of currRect
                    rect.width = currRect.x + currRect.width - rect.x;
                }
                else {  // new rect right of currRect
                    rect.width = rect.x + rect.width - currRect.x;
                    rect.x = currRect.x;
                }
            }
            else {
                // if currRect is bigger, ignore the new rect
                if (currRect.width * currRect.height > rect.width*rect.height) {
                    return;
                }
            }
        }
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
        while (ndx<cnt && ndx<profile.hvParam.maxLineCandidates) {
            if (Math.abs(candidates[ndx].x-lc.x)<=profile.hvParam.minLineGap) {
                // we found our candidate
                double newLength = lc.length+candidates[ndx].length;
                candidates[ndx].x = candidates[ndx].x * candidates[ndx].length/newLength + lc.x * lc.length/newLength;
                candidates[ndx].angle = candidates[ndx].angle * candidates[ndx].length/newLength + lc.angle * lc.length/newLength;
                candidates[ndx].length = newLength;
                return;
            }
            ndx++;
        }
        if (ndx<profile.hvParam.maxLineCandidates) {
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
        double avg = (profile.hvParam.finalDist *profile.hvParam.finalWidth + profile.hvParam.awayDist *profile.hvParam.awayWidth)/2;
        return avg / lastResult.width;
    }
}
