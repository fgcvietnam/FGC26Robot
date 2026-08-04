package fgc.vietnam.robot01.Hardware
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor


class Vision(hardwareMap: HardwareMap) {
    private val aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .build()
    private val portal: VisionPortal? = VisionPortal.Builder()
        .setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "Webcam"))
        .addProcessor(aprilTag)
        .build()

    fun getBestDetection(): AprilTagDetection? {
        val detections = aprilTag.detections
        if (detections.isEmpty()) return null
        return detections[0]
    }

    fun getRobotPose(detection: AprilTagDetection): Pose2d? {
        val d = getBestDetection() ?: return null

        if (d.robotPose == null) return null

        return Pose2d(
            d.robotPose.position.x,
            d.robotPose.position.y,
            d.robotPose.orientation.getYaw(AngleUnit.RADIANS)

        )
    }
}