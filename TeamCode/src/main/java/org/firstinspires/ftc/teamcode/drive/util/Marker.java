package org.firstinspires.ftc.teamcode.drive.util;

public class Marker {
    private int afterWaypointIndex;
    private Runnable runnable;

    public Marker(int afterWaypointIndex, Runnable run) {
        this.afterWaypointIndex = afterWaypointIndex;
        runnable = run;
    }

    public void start() {
        new Thread(runnable);
    }

    public void startSync() {
        runnable.run();
    }

    public int getAfterWaypointIndex() {
        return afterWaypointIndex;
    }
}
