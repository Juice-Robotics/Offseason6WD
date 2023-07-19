package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.vision.AprilTags;

public class Robot {
    //SUBSYSTEMS
    AprilTags aprilTags;


    public Robot(HardwareMap map) {
        aprilTags = new AprilTags(map.get(WebcamName.class, "Webcam 1"));
    }
}
