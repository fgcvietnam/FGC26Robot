package fgc.vietnam.robot01.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import fgc.vietnam.robot01.Config.VisionConfig;
import fgc.vietnam.robot01.Hardware.Vision;

public class PoseEstimator {

    private Pose2d estimatedPose;
    public PoseEstimator(Pose2d initialPose) {
        estimatedPose = initialPose;
    }
    public Pose2d getPose() {
        return estimatedPose;
    }
    /**
     * Called every loop.
     */
    public void updateOdometry(Pose2d encoderPose, Rotation2d imuHeading) {
        estimatedPose = new Pose2d(encoderPose.position, imuHeading);
    }

    /**
     * Called whenever an AprilTag measurement arrives.
     */
    public void addVisionMeasurement(Pose2d visionPose, AprilTagDetection detection) {

        if (visionPose == null)
            return;

        if (!shouldAccept(detection))
            return;

        double error =
                estimatedPose.position.minus(visionPose.position).norm();

        if (error > VisionConfig.MAX_VISION_ERROR) {
            estimatedPose = visionPose;
            return;
        }

        estimatedPose = interpolate(
                estimatedPose,
                visionPose,
                getVisionWeight(detection)
        );
    }

    public void resetPose(Pose2d pose) {
        estimatedPose = pose;
    }

    /**
     * Replace this with EKF later.
     */
    private Pose2d interpolate(Pose2d current,
                               Pose2d target,
                               double alpha) {

        double x =
                current.position.x * (1.0 - alpha)
                        + target.position.x * alpha;

        double y =
                current.position.y * (1.0 - alpha)
                        + target.position.y * alpha;

        double heading =
                current.heading.toDouble() * (1.0 - alpha)
                        + target.heading.toDouble() * alpha;

        return new Pose2d(
                x,
                y,
                heading
        );
    }

    private boolean shouldAccept(
            AprilTagDetection d
    ) {

        if (d.metadata == null)
            return false;

        if (d.ftcPose == null)
            return false;

        if (d.decisionMargin < 50)
            return false;

        if (d.ftcPose.range > 120)
            return false;

        return true;
    }

    private double getVisionWeight(AprilTagDetection d) {

        double distance = d.ftcPose.range;

        // 0.30 when close, 0.05 when far
        double alpha = VisionConfig.ALPHA_CLOSE - distance / 200.0;
        return Math.max(VisionConfig.ALPHA_FAR, Math.min(alpha, 0.30));
    }
}