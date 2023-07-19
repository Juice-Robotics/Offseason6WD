package org.firstinspires.ftc.teamcode.drive.builder;

import org.firstinspires.ftc.teamcode.drive.util.Path;
import org.firstinspires.ftc.teamcode.drive.util.Pose2d;

public class PathBuilder {
    Path path = new Path();

    public PathBuilder addWaypoint(Pose2d destination) {
        path.addWaypoint(destination);
        return this;
    }

    public PathBuilder addDisplacementMarker(Runnable markerContents) {
        path.addAfterWaypointMarker(path.getWaypoints().size() - 1, markerContents);
        return this;
    }

    public Path build() {
        return path;
    }
}
