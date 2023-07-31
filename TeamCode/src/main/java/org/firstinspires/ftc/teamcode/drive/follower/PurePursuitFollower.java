package org.firstinspires.ftc.teamcode.drive.follower;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.util.Path;
import org.firstinspires.ftc.teamcode.drive.util.Pose2d;

import java.util.ArrayList;


@Config
public class PurePursuitFollower {
    ArrayList<Path> paths;

    DcMotorEx left1;
    DcMotorEx left2;
    DcMotorEx right1;
    DcMotorEx right2;

    Runnable pathLoop = () -> {};
    public Pose2d currentPos;
    private PurePursuitCalculator pp = new PurePursuitCalculator();
    private boolean async = false;

    public static double kpLin = 20.0;
    public static double kpTurn = 1.0;
    public static double maxVelocity = 2.0;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public PurePursuitFollower(DcMotorEx left1, DcMotorEx left2, DcMotorEx right1, DcMotorEx right2) {
        this.left1 = left1;
        this.left2 = left2;
        this.right1 = right1;
        this.right2 = right2;

        this.left1.setVelocityPIDFCoefficients(p, i, d, f);
        this.left2.setVelocityPIDFCoefficients(p, i, d, f);
        this.right1.setVelocityPIDFCoefficients(p, i, d, f);
        this.right2.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public PurePursuitFollower(DcMotorEx left1, DcMotorEx left2, DcMotorEx right1, DcMotorEx right2, Runnable pathLoop) {
        this.left1 = left1;
        this.left2 = left2;
        this.right1 = right1;
        this.right2 = right2;
        this.pathLoop = pathLoop;

        this.left1.setVelocityPIDFCoefficients(p, i, d, f);
        this.left2.setVelocityPIDFCoefficients(p, i, d, f);
        this.right1.setVelocityPIDFCoefficients(p, i, d, f);
        this.right2.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void followPath(Path path) {
        async = false;
        paths.add(path);

        // SYNC BLOCKING
        while (!paths.get(0).finished()) {
            this.update();
            pathLoop.run();
        }

        paths.remove(0);
    }

    public void followPathAsync(Path path) {
        async = true;
        paths.add(path);
    }

    public void update() {
        Path path = paths.get(0);
        Pose2d lookahead = pp.calculate(path, currentPos);

        if (lookahead.heading == currentPos.heading && lookahead.x == currentPos.x && lookahead.y == currentPos.y) {
            path.setFinished();
            if (async) {
                paths.remove(0);
            }
        } else {
            // calculate errors
            double linearError = Math.sqrt(Math.pow((lookahead.getY() - currentPos.getY()), 2) + Math.pow((lookahead.getX() - currentPos.getX()), 2));
            double turnError = lookahead.getHeadingDiff(currentPos);

            //calculate velocities
            double linearVelocity = linearError * kpLin;
            double turnVelocity = turnError * kpTurn;

            //update drive motors
            setLeftMotors(linearVelocity - turnVelocity);
            setRightMotors(linearVelocity + turnVelocity);
        }

        // TODO: HANDLE WAYPOINTS
    }

    public void setLeftMotors ( double velocity){
        left1.setVelocity(velocity);
        left2.setVelocity(velocity);
    }

    public void setRightMotors ( double velocity){
        right1.setVelocity(velocity);
        right2.setVelocity(velocity);
    }

    public void setPoseEstimate(Pose2d pose) {
        currentPos = pose;
    }
}
