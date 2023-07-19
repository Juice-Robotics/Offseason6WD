package org.firstinspires.ftc.teamcode.drive.util;

public class Pose2d {
    private double x;
    private double y;
    private double heading; //DEGREES

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
