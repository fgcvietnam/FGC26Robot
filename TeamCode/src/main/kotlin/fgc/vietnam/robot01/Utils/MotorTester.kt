import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs

class MotorTester(
    private val motor: DcMotorEx,
    var expectedDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    private val power: Double = 0.40,
    private val testTimeMs: Int = 150,
    private val minEncoderTicks: Int = 25,
    private val maxCurrent: Double = 5.0,
) {

    enum class State {
        IDLE,
        RUNNING,
        FINISHED
    }

    data class Result(
        val passed: Boolean,
        val encoderWorking: Boolean,
        val directionCorrect: Boolean,
        val stalled: Boolean,
        val encoderDelta: Int,
        val current: Double
    )

    private val timer = ElapsedTime()

    private var state = State.IDLE
    private var startPosition = 0

    var result: Result? = null
        private set

    val passed: Boolean
        get() = result?.passed == true

    fun start() {
        startPosition = motor.currentPosition
        motor.power = power
        timer.reset()
        state = State.RUNNING
    }

    fun update() {
        if (state != State.RUNNING) return

        if (timer.milliseconds() < testTimeMs) return

        motor.power = 0.0

        val delta = motor.currentPosition - startPosition
        val current = motor.getCurrent(CurrentUnit.AMPS)

        val encoderWorking = abs(delta) >= minEncoderTicks
        val directionCorrect = when (expectedDirection) {
            DcMotorSimple.Direction.FORWARD -> delta > 0
            DcMotorSimple.Direction.REVERSE -> delta < 0
        }

        val stalled =
            current > maxCurrent &&
                    abs(delta) < minEncoderTicks

        val passed =
            encoderWorking &&
                    directionCorrect &&
                    !stalled

        result = Result(
            passed = passed,
            encoderWorking = encoderWorking,
            directionCorrect = directionCorrect,
            stalled = stalled,
            encoderDelta = delta,
            current = current
        )

        state = State.FINISHED
    }


    fun finished(): Boolean =
        state == State.FINISHED
}