package org.firstinspires.ftc.teamcode.drive.util;

public class Pose2d {
    public double x;
    public double y;
    public double heading; //DEGREES

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

    public double getHeadingDiff(Pose2d pose2){
        double turnError = pose2.heading - this.heading;
        if (turnError > 180 || turnError <-180) {
            turnError = -1 * java.lang.Math.signum(turnError) * (360 - Math.abs(turnError));
        }
        return turnError;
    }
}
