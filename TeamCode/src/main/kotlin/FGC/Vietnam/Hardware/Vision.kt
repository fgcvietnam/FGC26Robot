package FGC.Vietnam.Hardware

import android.graphics.Bitmap
import android.graphics.Canvas
import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import FGC.Vietnam.Config.VisionConfig
import global.first.IgnitingInnovationGameDatabase
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.opencv.android.Utils
import org.opencv.core.Mat
import java.util.concurrent.atomic.AtomicReference

internal class CameraStreamProcessor : VisionProcessor, CameraStreamSource {
    private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565))
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        val bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)
        Utils.matToBitmap(frame, bitmap)
        lastFrame.set(bitmap)
        return null
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>) {
        continuation.dispatch { bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()) }
    }
}

internal class Vision(hardwareMap: HardwareMap) {
    val streamProcessor = CameraStreamProcessor()

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
        .addProcessors(aprilTag, streamProcessor)
        .setCameraResolution(Size(640, 480))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .build()

    init {
        try {
            FtcDashboard.getInstance().startCameraStream(streamProcessor, 30.0)
        } catch (_: Exception) {
        }
    }

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
        try {
            FtcDashboard.getInstance().stopCameraStream()
        } catch (_: Exception) {
        }
        portal?.close()
    }
}