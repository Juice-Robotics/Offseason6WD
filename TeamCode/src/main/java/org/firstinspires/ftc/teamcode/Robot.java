package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.follower.PurePursuitFollower;
import org.firstinspires.ftc.teamcode.subsystems.vision.AprilTags;

public class Robot {
    //SUBSYSTEMS
    AprilTags aprilTags;
    PurePursuitFollower drive;

    public Robot(HardwareMap map) {
        aprilTags = new AprilTags(map.get(WebcamName.class, "Webcam 1"));
        drive = new PurePursuitFollower(
                map.get(DcMotorEx.class, "left1"),
                map.get(DcMotorEx.class, "left2"),
                map.get(DcMotorEx.class, "right1"),
                map.get(DcMotorEx.class, "right2"));
    }
}
