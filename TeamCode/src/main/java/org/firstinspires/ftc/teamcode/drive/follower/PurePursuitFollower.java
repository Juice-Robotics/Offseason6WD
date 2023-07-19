package org.firstinspires.ftc.teamcode.drive.follower;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.util.Path;

import java.util.ArrayList;

public class PurePursuitFollower {
    ArrayList<Path> paths;

    DcMotorEx left1;
    DcMotorEx left2;
    DcMotorEx right1;
    DcMotorEx right2;

    Runnable pathLoop = () -> {};

    public PurePursuitFollower(DcMotorEx left1, DcMotorEx left2, DcMotorEx right1, DcMotorEx right2) {
        this.left1 = left1;
        this.left2 = left2;
        this.right1 = right1;
        this.right2 = right2;
    }

    public PurePursuitFollower(DcMotorEx left1, DcMotorEx left2, DcMotorEx right1, DcMotorEx right2, Runnable pathLoop) {
        this.left1 = left1;
        this.left2 = left2;
        this.right1 = right1;
        this.right2 = right2;
        this.pathLoop = pathLoop;
    }

    public void followPath(Path path) {
        paths.add(path);

        // SYNC BLOCKING
        while (!paths.get(0).finished()) {
            this.update(paths.get(0));
            pathLoop.run();
        }

        paths.remove(0);
    }

    public void update(Path path) {

    }
}
