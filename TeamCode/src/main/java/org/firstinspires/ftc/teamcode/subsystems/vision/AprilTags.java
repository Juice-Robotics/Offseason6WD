package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import util.Pose3d;

public class AprilTags {
    AprilTagProcessor processor;
    VisionPortal visionPortal;
    AprilTagDetection aprilTagDetection;
    List<AprilTagDetection> aprilTagDetections;
    int aprilTagID;

    double cameraOffsetX = 0.0;
    double cameraOffsetY= 0.0;
    double cameraOffsetRotation = 0.0;

    public AprilTags(WebcamName camera) {
        processor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(processor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();
    }

    public void detect() {
        aprilTagDetections = processor.getDetections();

        for (AprilTagDetection laprilTagDetection : aprilTagDetections) {
            if (laprilTagDetection.metadata != null) {
                aprilTagID = laprilTagDetection.id;
                aprilTagDetection = laprilTagDetection;
            }
        }
    }

    public AprilTagPoseFtc getRelativePose() {
        return aprilTagDetection.ftcPose;
    }

    public Pose3d getAbsolutePose3d(Pose3d robotPose) {
        Pose3d tagPose = new Pose3d(robotPose.x, robotPose.y, robotPose.z, robotPose.rotation, robotPose.yaw);
        AprilTagPoseFtc tagRPose = getRelativePose();
        tagPose.x += tagRPose.x + cameraOffsetX;
        tagPose.y += tagRPose.y + cameraOffsetY;
        tagPose.z = tagRPose.z;
        tagPose.rotation += tagRPose.pitch + cameraOffsetRotation;
        tagPose.yaw = tagRPose.yaw;

        return tagPose;
    }
}
