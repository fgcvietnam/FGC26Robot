package RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import fgc.vietnam.robot01.Config.VisionConfig;

public class PoseEstimator {

    private Pose2d estimatedPose;
    private Pose2d lastOdometryPose;

    // Allows the pose heading to have an arbitrary field-frame heading
    // while the IMU itself remains zeroed at robot initialization.
    private double imuHeadingOffset = 0.0;

    public PoseEstimator(Pose2d initialPose) {
        this.estimatedPose = initialPose;
        this.lastOdometryPose = initialPose;
    }

    public Pose2d getPose() {
        return estimatedPose;
    }

    public void updateOdometry(
            Pose2d odometryPose,
            Rotation2d imuHeading) {

        double dx =
                odometryPose.position.x
                        - lastOdometryPose.position.x;

        double dy =
                odometryPose.position.y
                        - lastOdometryPose.position.y;

        // Convert IMU-relative heading into our field-frame heading.
        double heading =
                normalizeAngle(
                        imuHeading.toDouble()
                                + imuHeadingOffset
                );

        estimatedPose = new Pose2d(
                estimatedPose.position.x + dx,
                estimatedPose.position.y + dy,
                heading
        );

        lastOdometryPose = odometryPose;
    }

    public void addVisionMeasurement(
            Pose2d visionPose,
            AprilTagDetection detection) {

        if (visionPose == null)
            return;

        if (!shouldAccept(detection))
            return;

        double error =
                estimatedPose.position
                        .minus(visionPose.position)
                        .norm();

        // Reject obviously bad measurements.
        if (error > VisionConfig.MAX_VISION_ERROR)
            return;

        estimatedPose = interpolate(
                estimatedPose,
                visionPose,
                getVisionWeight(detection)
        );
    }

    public void resetPose(Pose2d pose) {

        estimatedPose = pose;
        lastOdometryPose = pose;

        /*
         * The IMU currently reports some heading H_imu.
         *
         * We want the estimator to report pose.heading.
         *
         * Therefore:
         *
         *     fieldHeading = imuHeading + offset
         *
         * so:
         *
         *     offset = desiredHeading - imuHeading
         *
         * The current IMU value is supplied separately through
         * setImuHeading().
         */
    }

    public void resetPose(
            Pose2d pose,
            Rotation2d currentImuHeading) {

        estimatedPose = pose;
        lastOdometryPose = pose;

        imuHeadingOffset =
                normalizeAngle(
                        pose.heading.toDouble()
                                - currentImuHeading.toDouble()
                );
    }

    private Pose2d interpolate(
            Pose2d current,
            Pose2d target,
            double alpha) {

        double x =
                current.position.x * (1.0 - alpha)
                        + target.position.x * alpha;

        double y =
                current.position.y * (1.0 - alpha)
                        + target.position.y * alpha;

        double currentHeading =
                current.heading.toDouble();

        double targetHeading =
                target.heading.toDouble();

        double headingError =
                normalizeAngle(
                        targetHeading - currentHeading
                );

        double heading =
                normalizeAngle(
                        currentHeading
                                + headingError * alpha
                );

        return new Pose2d(
                x,
                y,
                heading
        );
    }

    private boolean shouldAccept(
            AprilTagDetection d) {

        if (d == null)
            return false;

        if (d.metadata == null)
            return false;

        if (d.ftcPose == null)
            return false;

        if (d.decisionMargin < 50)
            return false;

        if (d.ftcPose.range > 2000)
            return false;

        return true;
    }

    private double getVisionWeight(
            AprilTagDetection d) {

        double distanceMm = d.ftcPose.range;

        double closeDistanceMm = 1000.0;
        double farDistanceMm = 2000.0;

        double t =
                (distanceMm - closeDistanceMm)
                        / (farDistanceMm - closeDistanceMm);

        t = Math.max(0.0, Math.min(t, 1.0));

        return VisionConfig.ALPHA_CLOSE * (1.0 - t)
                + VisionConfig.ALPHA_FAR * t;
    }

    private double normalizeAngle(double angle) {

        while (angle > Math.PI)
            angle -= 2.0 * Math.PI;

        while (angle < -Math.PI)
            angle += 2.0 * Math.PI;

        return angle;
    }
}