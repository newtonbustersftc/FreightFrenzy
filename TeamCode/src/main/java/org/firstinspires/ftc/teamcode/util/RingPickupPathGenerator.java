package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.apache.commons.math3.geometry.euclidean.twod.Segment;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.Logger;

import java.util.ArrayList;
import java.util.Comparator;

public class RingPickupPathGenerator {
    static TrajectoryBuilder currBuilder;
    static DriveConstraints constraints = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
    static double lastHeading = 0;
    public static double FIELD_WIDTH = 144.0; // 12'
    static double ROBOT_WIDTH = 18.0;
    Pose2d startPose;
    Pose2d endPose;
    static Line[] walls = new Line[4];

    static {
        // TOP
        walls[0] = new Line(new Vector2D(FIELD_WIDTH/2, FIELD_WIDTH/3), new Vector2D(FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);
        // LEFT
        walls[1] = new Line(new Vector2D(FIELD_WIDTH/2, FIELD_WIDTH/3), new Vector2D(-FIELD_WIDTH/2, FIELD_WIDTH/3), 0);
        // BOTTOM
        walls[2] = new Line(new Vector2D(-FIELD_WIDTH/2, FIELD_WIDTH/3), new Vector2D(-FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);
        // RIGHT
        walls[3] = new Line(new Vector2D(-FIELD_WIDTH/2, -FIELD_WIDTH/2), new Vector2D(FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);
    }

    static double NEAR_WALL_EXTRA = ROBOT_WIDTH * 3/2;

    public RingPickupPathGenerator(Pose2d startPose, Pose2d endPose) {
        this.startPose = startPose;
        this.endPose = endPose;
    }

    /**
     * We are at starting p1, need to add the move to p2 with p3 as next point
     */
    void addMove(Pose2d p1, Vector2d p2, Vector2d p3) {
        Logger.logFile("Add Move " + p1 + " -> " + p2 + " -> " + p3);
        Line p1ToP2 = new Line(new Vector2D(p1.getX(), p1.getY()), new Vector2D(p2.getX(), p2.getY()), 0);
        Line p2ToP3 = new Line(new Vector2D(p2.getX(), p2.getY()), new Vector2D(p3.getX(), p3.getY()), 0 );

        Line p2ToP1 = new Line(toVector2D(p2), toVector2D(p1), 0);
        double d1 = Math.min(p2.distTo(p1.vec()), ROBOT_WIDTH*2);
        double d2 = Math.min(p3.distTo(p2), ROBOT_WIDTH*2);
        double ang1 = p2ToP1.getAngle();
        double ang2 = p2ToP3.getAngle();
        Vector2D tmpP1 = new Vector2D(p2.getX() + d2 * Math.cos(ang1), p2.getY() + d2 * Math.sin(ang1));
        Vector2D tmpP2 = new Vector2D(p2.getX() + d1 * Math.cos(ang2), p2.getY() + d1 * Math.sin(ang2));
        Segment seg = new Segment(tmpP1, tmpP2, new Line(tmpP1, tmpP2, 0));
        lastHeading = seg.getLine().getAngle();
        Vector2D tmpP1a = new Vector2D(p1.getX() + Math.cos(p1.getHeading()), p1.getY() + Math.sin(p1.getHeading()));
        double d1p = tmpP1a.distance(toVector2D(p1));
        boolean reverse = (currBuilder==null)?(d1p>d1):false;   // only first segment allow reverse

        if (currBuilder==null) {
            currBuilder = new TrajectoryBuilder(p1, reverse, constraints);
            Logger.logFile("ADS Spline from " + p1);
        }
        if (reverse) {  // make sure it get the angle right if had to reverse on first leg
            Pose2d preP2 = new Pose2d(p2.getX() - Math.cos(lastHeading) * ROBOT_WIDTH/2, p2.getY() - Math.sin(lastHeading)*ROBOT_WIDTH/2, lastHeading);
            currBuilder.splineToSplineHeading(preP2, lastHeading);
            Logger.logFile("ADS Spline to " + preP2 + " reverse");
        }
        currBuilder.splineToSplineHeading(new Pose2d(p2.getX(), p2.getY(), lastHeading), lastHeading);
        Logger.logFile("ADS Spline to " + p2);
    }

    /**
     * Generate trajectory from startPos to ring 1, ring 2, ring 3 to endPos
     * @param r1 Ring 1 coordinate
     * @param r2 Ring 2 coordinate
     * @param r3 Ring 3 coordinate
     * @return Trajectory
     */
    ArrayList<Trajectory> pickUpAndShoot(Vector2d r1, Vector2d r2, Vector2d r3) {
        Logger.logFile("pickUpAndShoot " + r1 + "," + r2 + ", " + r3);
        ArrayList<Trajectory> moves = new ArrayList<Trajectory>();
        currBuilder = null;
        addMove(startPose, r1, r2);
        addMove(new Pose2d(r1.getX(), r1.getY(), lastHeading), r2, r3);
        addMove(new Pose2d(r2.getX(), r2.getY(), lastHeading), r3, endPose.vec());
        currBuilder.splineToSplineHeading(endPose, endPose.getHeading());
        moves.add(currBuilder.build());
        return moves;
    }

    /**
     * Define a line segment from the current robot position to shooting position, select
     * the rings that are close to this line segment.  If ring is too close to the wall, add
     * distance adjustment to that ring before ranking
     * @return selected rings
     */
    ArrayList<Vector2d> selectRings(ArrayList<Vector2d> rings) {
        Line segLine = new Line(new Vector2D(startPose.getX(), startPose.getY()), new Vector2D(endPose.getX(), endPose.getY()), 0);
        Segment seg = new Segment(new Vector2D(startPose.getX(), startPose.getY()), new Vector2D(endPose.getX(), endPose.getY()), segLine);
        // sort the rings by their distance to the segment
        DistToSeg comp = new DistToSeg(walls, seg);

        ArrayList<Vector2d> tmp = new ArrayList<Vector2d>(rings);
        tmp.sort(comp);
        ArrayList<Vector2d> resp = new ArrayList<Vector2d>();
        for(int i=0; i<Math.min(6, tmp.size()); i++) {
            resp.add(tmp.get(i));
        }
        return resp;
    }

    Vector2D toVector2D(Pose2d p) {
        return new Vector2D(p.getX(), p.getY());
    }

    Vector2D toVector2D(Vector2d v) {
        return new Vector2D(v.getX(), v.getY());
    }

    Segment toSegment(Vector2D v1, Vector2D v2) {
        return new Segment(v1, v2, new Line(v1, v2, 0.0));
    }

    Vector2D move(Pose2d pose, double dist) {
        double x = pose.getX() + dist * Math.cos(pose.getHeading());
        double y = pose.getY() + dist * Math.sin(pose.getHeading());
        return new Vector2D(x, y);
    }

    // Enumerate all chosen ring possible pick up order and get quickest one
    public ArrayList<Trajectory> generatePath(ArrayList<Vector2d> rings) {
        Vector2D newStart = move(startPose, ROBOT_WIDTH);
        Vector2D newEnd = move(endPose, -ROBOT_WIDTH);
        ArrayList<Vector2d> chosen = selectRings(rings);
        Logger.logFile("GeneratePath rings - " + chosen.size());
        for(Vector2d v : chosen) {
            Logger.logFile("Ring:" + v.getX() + ", " + v.getY());
        }
        // build a path for different pick up orders
        ArrayList<Trajectory> bestMove = new ArrayList<Trajectory>();
        int moveExamined = 0;
        long startTime = System.currentTimeMillis();
        double bestCost = 10000;
        int bestA = 1, bestB = 2, bestC = 3;
        // if number of rings seen is not 3
        if (chosen.size()==2) {
            chosen.add(new Vector2d(endPose.getX()-19, endPose.getY()));
        }
        if (chosen.size()==1) {
            Vector2d r = chosen.get(0);
            Line p2ToP1 = new Line(toVector2D(r), toVector2D(startPose), 0);
            Line p2ToP3 = new Line(toVector2D(r), toVector2D(endPose), 0 );
            double ang1 = p2ToP1.getAngle();
            double ang2 = p2ToP3.getAngle();

            chosen.add(new Vector2d(r.getX() + Math.cos(ang1), r.getY() + Math.sin(ang1)));
            chosen.add(new Vector2d(r.getX() + Math.cos(ang1)*2, r.getY() + Math.sin(ang1)*2));
        }
        if (chosen.size()==0) {
            chosen.add(new Vector2d(endPose.getX()+5, endPose.getY()+5));
            chosen.add(new Vector2d(endPose.getX()-5, endPose.getY()));
            chosen.add(new Vector2d(endPose.getX()-8, endPose.getY()));
        }
        for (int a = 0; a < chosen.size(); a++) {
            for (int b = 0; b < chosen.size(); b++) {
                if (a != b) {
                    for (int c = 0; c < chosen.size(); c++) {
                        if (a != b && a != c && b != c) {
                            double currCost = newStart.distance(toVector2D(chosen.get(a))) +
                                    chosen.get(a).distTo(chosen.get(b)) +
                                    chosen.get(b).distTo(chosen.get(c)) +
                                    newEnd.distance(toVector2D(chosen.get(c)));
                            if (isNearWall(chosen.get(a))>0) {
                                currCost += NEAR_WALL_EXTRA;
                            }
                            if (isNearWall(chosen.get(b))>0) {
                                currCost += NEAR_WALL_EXTRA;
                            }
                            if (isNearWall(chosen.get(c))>0) {
                                currCost += NEAR_WALL_EXTRA;
                            }
                            moveExamined++;

                            if (bestCost > currCost) {
                                bestCost = currCost;
                                bestA = a; bestB = b; bestC = c;
                            }
                        }
                    }
                }
            }
        }
        long timeSpent = System.currentTimeMillis() - startTime;
        Logger.logFile("Moves compared: " + moveExamined + " time: " + timeSpent + " avg: " + 1.0*timeSpent/moveExamined);
        Logger.logFile("rings.add(new Vector2d" + chosen.get(bestA) + ");");    // to be used in simulator if needed
        Logger.logFile("rings.add(new Vector2d" + chosen.get(bestB) + ");");
        Logger.logFile("rings.add(new Vector2d" + chosen.get(bestC) + ");");
        bestMove = pickUpAndShoot(chosen.get(bestA), chosen.get(bestB), chosen.get(bestC));
        return bestMove;
    }

    int isNearWall(Vector2d r) {
        for(int i=0; i<walls.length; i++) {
            if (walls[i].distance(toVector2D(r)) < ROBOT_WIDTH*5/8) {     // if too close to the wall, add adjustment
                return i + 1;
            }
        }
        return 0;
    }

    class DistToSeg implements Comparator<Vector2d> {
        Segment seg;
        Line[] walls;

        public DistToSeg(Line[] walls, Segment seg) {
            this.seg = seg;
            this.walls = walls;
        }

        @Override
        /**
         * Sort by the distance of ring to the line segment.  If ring is too close to wall, distance increases by Robot Width
         */
        public int compare(Vector2d a, Vector2d b) {
            Vector2D a2D = new Vector2D(a.getX(), a.getY());
            Vector2D b2D = new Vector2D(b.getX(), b.getY());

            double aX = 0, bX = 0;
            for (int i = 0; i < walls.length; i++) {
                if (walls[i].distance(a2D) < ROBOT_WIDTH / 2) {     // if too close to the wall, add adjustment
                    aX = ROBOT_WIDTH;
                }
                if (walls[i].distance(b2D) < ROBOT_WIDTH / 2) {     // if too close to the wall, add adjustment
                    bX = ROBOT_WIDTH;
                }
            }
            return (int) Math.signum((seg.distance(a2D) + aX) - (seg.distance(b2D) + bX));
        }
    }
}
