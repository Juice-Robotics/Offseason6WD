package org.firstinspires.ftc.teamcode.drive.util;

import java.util.ArrayList;

public class Path {
    private ArrayList<Pose2d> waypoints;
    private ArrayList<Marker> markers;
    private boolean finished = false;

    public void addWaypoint(Pose2d pose) {
        waypoints.add(pose);
    }

    public void addAfterWaypointMarker(int afterWaypointIndex, Runnable runnable) {
        markers.add(new Marker(afterWaypointIndex, runnable));
    }

    public void setFinished() {
        finished = true;
    }

    public ArrayList<Pose2d> getWaypoints() {
        return waypoints;
    }

    public ArrayList<Marker> getMarkers() {
        return markers;
    }

    public boolean finished() {
        return true;
    }
}
