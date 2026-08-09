package FGC.Vietnam.Config

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

@Config
object VisionConfig {
    @JvmField var MAX_VISION_ERROR: Double = 70.0 //mm
    @JvmField var VISION_BLEND: Double = 0.15 //mm
    @JvmField var ALPHA_FAR: Double = 0.05
    @JvmField var ALPHA_CLOSE: Double = 0.3

    val CAMERA_POSITION = Position(
        DistanceUnit.MM,
        100.0,  // camera forward from robot center
        0.0,    // camera left/right offset
        200.0,  // camera height
        0L
    )

    val CAMERA_ORIENTATION = YawPitchRollAngles(
        AngleUnit.DEGREES,
        0.0,
        20.0,
        0.0,
        0L
    )


}