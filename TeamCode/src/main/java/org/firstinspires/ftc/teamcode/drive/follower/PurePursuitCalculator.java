package org.firstinspires.ftc.teamcode.drive.follower;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.drive.util.Path;
import org.firstinspires.ftc.teamcode.drive.util.Pose2d;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class PurePursuitCalculator {
    public static double lookAheadDis = 0.5; //tune

    public Pose2d calculate(Path pathp, Pose2d currentPos) {
        Pose2d lookahead = null;
        ArrayList<Pose2d> path = pathp.getWaypoints();

        // iterate through all pairs of points
        for (int i = 0; i < path.size() - 1; i++) {
            // form a segment from each two adjacent points
            Pose2d segmentStart = path.get(i);
            Pose2d segmentEnd = path.get(i + 1);

            // translate the segment to the origin
            double[] p1 = new double[]{segmentStart.x - currentPos.x, segmentStart.y - currentPos.y};
            double[] p2 = new double[]{segmentEnd.x - currentPos.x, segmentEnd.y - currentPos.y};

            // calculate an intersection of a segment and a circle with radius r (lookahead) and origin (0, 0)
            double dx = p2[0] - p1[0];
            double dy = p2[1] - p1[1];
            double d = Math.sqrt(dx * dx + dy * dy);
            double D = p1[0] * p2[1] - p2[0] * p1[1];

            // if the discriminant is zero or the points are equal, there is no intersection
            double discriminant = lookAheadDis * lookAheadDis * d * d - D * D;
            if (discriminant < 0 || Arrays.equals(p1, p2)) continue;

            // the x components of the intersecting points
            double x1 = (D * dy + java.lang.Math.signum(dy) * dx * Math.sqrt(discriminant)) / (d * d);
            double x2 = (D * dy - java.lang.Math.signum(dy) * dx * Math.sqrt(discriminant)) / (d * d);

            // the y components of the intersecting points
            double y1 = (-D * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (d * d);
            double y2 = (-D * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (d * d);

            // whether each of the intersections are within the segment (and not the entire line)
            boolean validIntersection1 = Math.min(p1[0], p2[0]) < x1 && x1 < Math.max(p1[0], p2[0])
                    || Math.min(p1[1], p2[1]) < y1 && y1 < Math.max(p1[1], p2[1]);
            boolean validIntersection2 = Math.min(p1[0], p2[0]) < x2 && x2 < Math.max(p1[0], p2[0])
                    || Math.min(p1[1], p2[1]) < y2 && y2 < Math.max(p1[1], p2[1]);

            // remove the old lookahead if either of the points will be selected as the lookahead
            if (validIntersection1 || validIntersection2) {lookahead = null;}

            // select the first one if it's valid
            if (validIntersection1) {
                lookahead = new Pose2d(x1 + currentPos.x, y1 + currentPos.y, 0);
                break;
            }

            // select the second one if it's valid and either lookahead is none,
            // or it's closer to the end of the segment than the first intersection
            if (validIntersection2) {
                if (lookahead == null || Math.abs(x1 - p2[0]) > Math.abs(x2 - p2[0]) || Math.abs(y1 - p2[1]) > Math.abs(y2 - p2[1])) {
                    lookahead = new Pose2d(x2 + currentPos.x, y2 + currentPos.y,0);
                    break;
                }
            }
            // special case for the very last point on the path
            if (path.size() > 0) {
                Pose2d lastPoint = path.get(path.size() - 1);

                double endX = lastPoint.x;
                double endY = lastPoint.y;

                // if we are closer than lookahead distance to the end, set it as the lookahead
                if (Math.sqrt((endX - currentPos.x) * (endX - currentPos.x) + (endY - currentPos.y) * (endY - currentPos.y)) <= lookAheadDis) {
                    lookahead = new Pose2d(endX, endY,0);
                }
                break;
            }
        }

        // obtained goal point, now compute turn vel

        // calculate absTargetAngle with the atan2 function
        double absTargetAngle = java.lang.Math.atan2(lookahead.y - currentPos.y, lookahead.x - currentPos.x) * 180 / Math.PI;
        if (absTargetAngle< 0){absTargetAngle += 360;}
        lookahead.heading = absTargetAngle;


        return lookahead;

    }
}
