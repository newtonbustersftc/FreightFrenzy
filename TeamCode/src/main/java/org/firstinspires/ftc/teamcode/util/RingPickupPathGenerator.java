package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.apache.commons.math3.geometry.euclidean.twod.Segment;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;


import java.util.ArrayList;
import java.util.Comparator;

public class RingPickupPathGenerator {
    static TrajectoryBuilder currBuilder;
    static DriveConstraints constraints = new DriveConstraints(20.0, 15.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
    static double lastHeading = 0;
    public static double FIELD_WIDTH = 144.0; // 12'
    static double ROBOT_WIDTH = 18.0;
    Pose2d startPose;
    Pose2d endPose;
    static Line[] walls = new Line[4];

    static {
        walls[0] = new Line(new Vector2D(FIELD_WIDTH/2, FIELD_WIDTH/2), new Vector2D(FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);      // we don't have full
        walls[1] = new Line(new Vector2D(FIELD_WIDTH/2, FIELD_WIDTH/2), new Vector2D(-FIELD_WIDTH/2, FIELD_WIDTH/2), 0);
        walls[2] = new Line(new Vector2D(-FIELD_WIDTH/2, FIELD_WIDTH/2), new Vector2D(-FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);
        walls[3] = new Line(new Vector2D(-FIELD_WIDTH/2, -FIELD_WIDTH/2), new Vector2D(FIELD_WIDTH/2, -FIELD_WIDTH/2), 0);
    }

    public RingPickupPathGenerator(Pose2d startPose, Pose2d endPose) {
        this.startPose = startPose;
        this.endPose = endPose;
    }

    /**
     * We are at starting p1, need to add the move to p2 with p3 as next point
     */
    void addMove(Pose2d p1, Vector2d p2, Vector2d p3) {
        System.out.println("Add Move " + p1 + " -> " + p2 + " -> " + p3);
        Line p1ToP2 = new Line(new Vector2D(p1.getX(), p1.getY()), new Vector2D(p2.getX(), p2.getY()), 0);
        Line p2ToP3 = new Line(new Vector2D(p2.getX(), p2.getY()), new Vector2D(p3.getX(), p3.getY()), 0 );
        boolean reverse = Math.abs((p1.getHeading() - p1ToP2.getAngle()))>Math.PI/2;  // when the angle of more than 90 degree, reverse
        // The angle should try to pass P2 to P3
        if (Math.abs(p1ToP2.getAngle() - p2ToP3.getAngle())>Math.PI) {
            lastHeading = (p2ToP3.getAngle() - p1ToP2.getAngle())/2 + Math.signum(p1ToP2.getAngle() - p2ToP3.getAngle()) * Math.PI;
        }
        else {
            lastHeading = (p1ToP2.getAngle() + p2ToP3.getAngle()) / 2;
        }
        if (currBuilder==null) {
            currBuilder = new TrajectoryBuilder(p1, reverse, constraints);
        }
        currBuilder.splineToSplineHeading(new Pose2d(p2.getX(), p2.getY(), lastHeading), lastHeading);
    }

    ArrayList<Trajectory> pickUpAndShoot(Vector2d r1, Vector2d r2, Vector2d r3) {
        System.out.println("pickUpAndShoot " + r1 + "," + r2 + ", " + r3);
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
     * @return
     */
    ArrayList<Vector2d> selectRings(ArrayList<Vector2d> rings) {
        Line segLine = new Line(new Vector2D(startPose.getX(), startPose.getY()), new Vector2D(endPose.getX(), endPose.getY()), 0);
        Segment seg = new Segment(new Vector2D(startPose.getX(), startPose.getY()), new Vector2D(endPose.getX(), endPose.getY()), segLine);
        // sort the rings by their distance to the segment
        DistToSeg comp = new DistToSeg(walls, seg);

        ArrayList<Vector2d> tmp = new ArrayList<Vector2d>(rings);
        tmp.sort(comp);
        ArrayList<Vector2d> resp = new ArrayList<Vector2d>();
        for(int i=0; i<Math.min(4, tmp.size()); i++) {
            resp.add(tmp.get(i));
        }
        return resp;
    }

    // Enumerate all chosen ring possible pick up order and get quickest one
    public ArrayList<Trajectory> generatePath(ArrayList<Vector2d> rings) {
        ArrayList<Vector2d> chosen = selectRings(rings);
        System.out.println("GeneratePath rings - " + chosen.size());
        for(Vector2d v : chosen) {
            System.out.println("Ring:" + v.getX() + ", " + v.getY());
        }
        // build a path for different pick up orders
        ArrayList<Trajectory> bestMove = new ArrayList<Trajectory>();
        int moveExamined = 0;
        long startTime = System.currentTimeMillis();
        double bestDuration = 10000;
        for (int a = 0; a < chosen.size(); a++) {
            for (int b = 0; b < chosen.size(); b++) {
                if (a != b) {
                    for (int c = 0; c < chosen.size(); c++) {
                        if (a != b && a != c && b != c) {
                            double currDuration = 0;
                            moveExamined++;
                            ArrayList<Trajectory> move = pickUpAndShoot(chosen.get(a), chosen.get(b), chosen.get(c));
                            for(Trajectory t : move) {
                                currDuration = currDuration + t.duration();
                            }
                            if (bestDuration > currDuration) {
                                bestDuration = currDuration;
                                bestMove = move;
                            }
                        }
                    }
                }
            }
        }
        long timeSpent = System.currentTimeMillis() - startTime;
        System.out.println("Example " + moveExamined + " time: " + timeSpent + " avg: " + 1.0*timeSpent/moveExamined);
        return bestMove;
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