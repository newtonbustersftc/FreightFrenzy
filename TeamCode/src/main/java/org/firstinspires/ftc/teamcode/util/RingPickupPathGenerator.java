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

/**
 * Defines the parameter needs to calculate the robot poses to pick up a ring by the wall
 * Say ring is at position x=68, y=15, which is near the wall on the goal side, the robot will
 * first get to entry position -52, 15 with heading 0, and then move to final position -61, 15
 * with heading 0.
 * For Top wall:
 * multiplierX = 0, multiplierY=1, entryOffsetX=70-18=52, finalOffsetX=70-9=61, entryOffsetY=0, finalOffsetY=0
 * So robot entry position: x = -68 * 0 + 52 = 52, y = 15 * 1 + 0 = 15
 * Robot final position: x = -68 * 0 + 61 = 61, y = 15 * 1 + 0 = 15
 */
class WallHandleParam {
    double multiplierX;
    double multiplierY;
    double entryOffsetX;
    double entryOffsetY;
    double finalOffsetX;
    double finalOffsetY;
    double heading;

    public WallHandleParam(double mx, double my, double ex, double ey, double fx, double fy, double h) {
        multiplierX = mx;
        multiplierY = my;
        entryOffsetX = ex;
        entryOffsetY = ey;
        finalOffsetX = fx;
        finalOffsetY = fy;
        heading = h;
    }
}

public class RingPickupPathGenerator {
    static TrajectoryBuilder currBuilder;
    DriveConstraints constraints = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
    DriveConstraints constraintsWall = new DriveConstraints(20.0, 15.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
    static double lastHeading = 0;
    public static double FIELD_WIDTH = 140.0; // 60cm x 6 / 2.54 = 141.7
    static double ROBOT_WIDTH = 18.0;
    static double REVERSE_OFFSET = 0.0;
    Pose2d startPose;
    Pose2d endPose;
    static Line[] walls = new Line[4];
    static WallHandleParam[] whp = new WallHandleParam[4];  // Robot location offset and heading to pick up rings by each wall

    static {
        // TOP
        walls[0] = new Line(new Vector2D(FIELD_WIDTH/2, FIELD_WIDTH/6), new Vector2D(FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);
        // LEFT
        walls[1] = new Line(new Vector2D(FIELD_WIDTH/2, FIELD_WIDTH/6), new Vector2D(-FIELD_WIDTH/2, FIELD_WIDTH/6), 0);
        // BOTTOM
        walls[2] = new Line(new Vector2D(-FIELD_WIDTH/2, FIELD_WIDTH/6), new Vector2D(-FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);
        // RIGHT
        walls[3] = new Line(new Vector2D(-FIELD_WIDTH/2, -FIELD_WIDTH/2), new Vector2D(FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);

        whp[0] = new WallHandleParam(0, 1, FIELD_WIDTH/2 - ROBOT_WIDTH, 0, FIELD_WIDTH/2 - ROBOT_WIDTH/2 - REVERSE_OFFSET, 0, 0);
        whp[1] = new WallHandleParam(1, 0, 0, FIELD_WIDTH/6 - ROBOT_WIDTH, 0, FIELD_WIDTH/6 - ROBOT_WIDTH/2 - REVERSE_OFFSET, Math.PI/2);
        whp[2] = new WallHandleParam(0, 1, -FIELD_WIDTH/2 + ROBOT_WIDTH, 0, -FIELD_WIDTH/2 + ROBOT_WIDTH/2 + REVERSE_OFFSET, 0, Math.PI);
        whp[3] = new WallHandleParam(1, 0, 0, -FIELD_WIDTH/2 + ROBOT_WIDTH, 0, -FIELD_WIDTH/2 + ROBOT_WIDTH/2 + REVERSE_OFFSET, -Math.PI/2);
    }

    static double NEAR_WALL_EXTRA = ROBOT_WIDTH * 3/2;

    // The resulting trajectory array to pick up the rings
    ArrayList<Trajectory> moves = new ArrayList<Trajectory>();

    /**
     * Constructor
     * @param startPose current robot position
     * @param endPose final robot position
     */
    public RingPickupPathGenerator(Pose2d startPose, Pose2d endPose) {
        this.startPose = startPose;
        this.endPose = endPose;
    }

    /**
     * Overwrite the default drive constraints if needed
     * @param driveConstraints
     */
    public void setDriveConstraints(DriveConstraints driveConstraints) {
        if (driveConstraints!=null) {
            this.constraints = driveConstraints;
        }
    }

    /**
     * We are at starting p1, need to add the move to p2 with p3 as next point
     */
    void addMove(Pose2d p1, Vector2d p2, Vector2d p3) {
        Logger.logFile("Add Move " + p1 + " -> " + p2 + " -> " + p3);
        // check if p2 is next to a wall
        int w = isNearWall(p2);
        Vector2d p2f = null;
        if (w!=0) {
            // if near a wall, p2 will be the entrance position, p2f is the final position
            p2 = new Vector2d(p2.getX()*whp[w-1].multiplierX + whp[w-1].entryOffsetX, p2.getY()*whp[w-1].multiplierY + whp[w-1].entryOffsetY);
            p2f = new Vector2d(p2.getX()*whp[w-1].multiplierX + whp[w-1].finalOffsetX, p2.getY()*whp[w-1].multiplierY + whp[w-1].finalOffsetY);
            lastHeading = whp[w-1].heading;
        }
        else {
            // if not near a wall, calculate the optimal robot heading to pick up ring based on the position of robot,
            // the ring and where it needs to get to next
            Line p2ToP3 = new Line(new Vector2D(p2.getX(), p2.getY()), new Vector2D(p3.getX(), p3.getY()), 0 );
            Line p2ToP1 = new Line(toVector2D(p2), toVector2D(p1), 0);
            double d1 = Math.min(p2.distTo(p1.vec()), ROBOT_WIDTH*2);
            double d2 = Math.min(p3.distTo(p2), ROBOT_WIDTH*2);
            double ang1 = getAngle(toVector2D(p2), toVector2D(p1));
            double ang2 = getAngle(toVector2D(p2), toVector2D(p3));
            if (Math.abs(ang1-ang2)<Math.PI/6) {
                // special handling if robot comes in and need to back out with a sharp angle
                // then we should have the robot go straight pass the ring and do reverse to p3
                lastHeading = getAngle(toVector2D(p1), toVector2D(p2));
                p2f = p2;
                p2 = new Vector2d(p2f.getX() - ROBOT_WIDTH/2 * Math.cos(lastHeading), p2f.getY() - ROBOT_WIDTH/2 * Math.sin(lastHeading));
                lastHeading = getAngle(toVector2D(p1), toVector2D(p2));
            }
            else {
                Vector2D tmpP1 = new Vector2D(p2.getX() + d2 * Math.cos(ang1), p2.getY() + d2 * Math.sin(ang1));
                Vector2D tmpP2 = new Vector2D(p2.getX() + d1 * Math.cos(ang2), p2.getY() + d1 * Math.sin(ang2));
                lastHeading = getAngle(tmpP1, tmpP2);
            }
        }

        if (currBuilder==null) {    // first move from starting position
            // Decide if we start robot in Reverse - if robot go on current heading for 1 inch, and become farther away from next point
            // Find the point 1 inch away with current heading
            Vector2D tmpP1a = new Vector2D(p1.getX() + Math.cos(p1.getHeading()), p1.getY() + Math.sin(p1.getHeading()));
            // distance to next point
            double d1p = tmpP1a.distance(toVector2D(p2));
            boolean reverse = d1p>p2.distTo(p1.vec());
            currBuilder = new TrajectoryBuilder(p1, reverse, constraints);
            Logger.logFile("ADS Spline from " + p1 + (reverse?" Reverse" : ""));
            if (reverse && p2f==null) {
                // make sure it get the angle right if had to reverse on first leg, so we add a point before getting to the ring
                Pose2d preP2 = new Pose2d(p2.getX() - Math.cos(lastHeading) * ROBOT_WIDTH/2, p2.getY() - Math.sin(lastHeading)*ROBOT_WIDTH/2, lastHeading);
                currBuilder.splineToSplineHeading(preP2, lastHeading);
                Logger.logFile("ADS Spline to " + preP2 + " reverse");
            }
        }
        currBuilder.splineToSplineHeading(new Pose2d(p2, lastHeading), lastHeading);
        Logger.logFile("ADS Spline to " + p2);
        if (p2f != null) {
            currBuilder.splineToSplineHeading(new Pose2d(p2f, lastHeading), lastHeading, constraintsWall);
            moves.add(currBuilder.build());
            Logger.logFile("ADS Spline to " + p2f);
            currBuilder = new TrajectoryBuilder(new Pose2d(p2f, lastHeading), true, constraints);
            Logger.logFile("ADS Spline from " + p2f);
        }
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
        currBuilder = null;
        addMove(startPose, r1, r2);
        addMove(new Pose2d(r1.getX(), r1.getY(), lastHeading), r2, r3);
        addMove(new Pose2d(r2.getX(), r2.getY(), lastHeading), r3, endPose.vec());
        currBuilder.splineToSplineHeading(endPose, endPose.getHeading());
        moves.add(currBuilder.build());
        return moves;
    }

    ArrayList<Trajectory> pickUpAndShoot(Vector2d r1) {
        Logger.logFile("pickUpAndShoot " + r1);
        currBuilder = null;
        addMove(startPose, r1, endPose.vec());
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

    /**
     * Utility function to convert RoadRunner Pose2d object to Apache Math Vector2D object
     * @param p RoadRunner Pose2d object
     * @return Apache Math Vector2D object
     */
    Vector2D toVector2D(Pose2d p) {
        return new Vector2D(p.getX(), p.getY());
    }

    /**
     * Utility function to convert RoadRunner Vector2d object to Apache Math Vector2D object
     * @param v RoadRunner Vector2d object
     * @return Apache Math Vector2D object
     */
    Vector2D toVector2D(Vector2d v) {
        return new Vector2D(v.getX(), v.getY());
    }

    double getAngle(Vector2D p0, Vector2D p1) {
        return Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
    }

    /**
     * Utility function to create a Apache Math line segment from 2 Vector2D points with 0 tolerance
     * @param v1 point 1
     * @param v2 point 2
     * @return Line segment from v1 to v2
     */
    Segment toSegment(Vector2D v1, Vector2D v2) {
        return new Segment(v1, v2, new Line(v1, v2, 0.0));
    }

    /**
     * Utility function to calculate the position if move from current pose2d by distance
     * @param pose Current pose2d (location & Heading)
     * @param dist Distance to move
     * @return Final position
     */
    Vector2D move(Pose2d pose, double dist) {
        double x = pose.getX() + dist * Math.cos(pose.getHeading());
        double y = pose.getY() + dist * Math.sin(pose.getHeading());
        return new Vector2D(x, y);
    }

    /**
     * Enumerate all chosen ring possible pick up order and get quickest one
     * @return Array of RoadRunner Trajectory
     */
    public ArrayList<Trajectory> generatePath(ArrayList<Vector2d> rings) {
        Vector2D newStart = move(startPose, ROBOT_WIDTH);
        Vector2D newEnd = move(endPose, -ROBOT_WIDTH);
        Logger.logFile("GeneratePath rings - " + rings.size());
        // if number of rings seen is not 3
        if (rings.size()==2) {
            rings.add(new Vector2d(endPose.getX()-8, endPose.getY()));
        }
        else if (rings.size()==1) {
            // there's only one right, special processing
            return pickUpAndShoot(rings.get(0));
        }
        if (rings.size()==0) {
            // just have it go around and do nothing if see nothing to waste some time
            rings.add(new Vector2d(endPose.getX()+5, endPose.getY()+5));
            rings.add(new Vector2d(endPose.getX()-5, endPose.getY()));
            rings.add(new Vector2d(endPose.getX()-8, endPose.getY()));
        }
        ArrayList<Vector2d> chosen = selectRings(rings);
        for(Vector2d v : chosen) {
            Logger.logFile("Ring:" + v.getX() + ", " + v.getY());
        }
        // build a path for different pick up orders
        ArrayList<Trajectory> bestMove = new ArrayList<Trajectory>();
        int moveExamined = 0;
        long startTime = System.currentTimeMillis();
        double bestCost = 10000;
        int bestA = 1, bestB = 2, bestC = 3;
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

    /**
     * Is the ring near one of the wall defined
     * @param r Ring position
     * @return 0 - not near any wall.  1-4 is near one of walls
     */
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
