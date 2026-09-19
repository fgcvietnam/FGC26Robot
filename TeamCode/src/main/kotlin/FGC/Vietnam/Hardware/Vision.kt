package FGC.Vietnam.Hardware

import android.util.Size
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import FGC.Vietnam.Config.VisionConfig
import global.first.IgnitingInnovationGameDatabase
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

internal class Vision(hardwareMap: HardwareMap) {
    private val aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .setTagLibrary(IgnitingInnovationGameDatabase.getIgnitingInnovationTagLibrary())
        .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
        .setCameraPose(VisionConfig.CAMERA_POSITION, VisionConfig.CAMERA_ORIENTATION)
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setDrawTagOutline(true)
        .setDrawTagID(true)
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .build()

    private val portal: VisionPortal? = VisionPortal.Builder()
        .setCamera(hardwareMap.get<WebcamName>(WebcamName::class.java, "Webcam"))
        .addProcessor(aprilTag)
        .setCameraResolution(Size(640, 480))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .build()

    fun getBestDetection(): AprilTagDetection? {
        val detections = aprilTag.detections
        if (detections.isEmpty()) return null
        return detections[0]
    }

    fun getRobotPose(detection: AprilTagDetection): Pose2d? {
        val robotPose = detection.robotPose ?: return null
        return Pose2d(
            robotPose.position.x,
            robotPose.position.y,
            robotPose.orientation.getYaw(AngleUnit.RADIANS)
        )
    }

    fun stop() {
        portal?.close()
    }
}
