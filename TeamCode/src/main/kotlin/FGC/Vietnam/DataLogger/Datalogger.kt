package FGC.Vietnam.DataLogger

import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.io.BufferedWriter
import java.io.Closeable
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.io.OutputStreamWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

internal class Datalogger private constructor(
    val file: File,
    private val writer: BufferedWriter,
) : Closeable {
    private val queue = ArrayBlockingQueue<String>(QUEUE_CAPACITY)
    private val isRunning = AtomicBoolean(true)
    private val rowCounter = AtomicInteger(0)

    val rowCount: Int get() = rowCounter.get()

    @Volatile
    var errorMessage: String? = null
        private set

    private val writerThread = Thread({
        var unwrittenCount = 0
        while (isRunning.get() || !queue.isEmpty()) {
            try {
                val line = queue.poll(100, TimeUnit.MILLISECONDS) ?: continue
                writer.write(line)
                writer.newLine()
                unwrittenCount++

                if (unwrittenCount >= FLUSH_INTERVAL_ROWS || queue.isEmpty()) {
                    writer.flush()
                    unwrittenCount = 0
                }
            } catch (exception: Exception) {
                if (errorMessage == null) {
                    errorMessage = exception.message ?: exception.javaClass.simpleName
                }
            }
        }
        try {
            writer.flush()
        } catch (_: Exception) {}
    }, "Datalogger-Async-Writer").apply {
        isDaemon = true
        priority = Thread.MIN_PRIORITY
        start()
    }

    fun writeRow(values: List<Any?>): Boolean {
        if (errorMessage != null || !isRunning.get()) return false
        val line = values.joinToString(separator = ",", transform = ::escape)
        val offered = queue.offer(line)
        if (offered) {
            rowCounter.incrementAndGet()
            return true
        }
        return false
    }

    override fun close() {
        if (!isRunning.compareAndSet(true, false)) return
        try {
            writerThread.join(1000)
        } catch (_: InterruptedException) {
            Thread.currentThread().interrupt()
        }

        try {
            writer.close()
        } catch (exception: IOException) {
            if (errorMessage == null) {
                errorMessage = exception.message ?: exception.javaClass.simpleName
            }
        }
    }

    private fun escape(value: Any?): String {
        val text = value?.toString().orEmpty()
        if (text.none { it == ',' || it == '"' || it == '\n' || it == '\r' }) {
            return text
        }
        return "\"${text.replace("\"", "\"\"")}\""
    }

    internal companion object {
        private const val QUEUE_CAPACITY = 4096
        private const val FLUSH_INTERVAL_ROWS = 50

        fun create(prefix: String, header: List<String>): Datalogger {
            val directory = File(AppUtil.FIRST_FOLDER, "Datalogs")
            if (!directory.exists() && !directory.mkdirs()) {
                throw IOException("Unable to create datalog directory: ${directory.absolutePath}")
            }

            val timestamp = SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.US).format(Date())
            val file = File(directory, "${prefix}_$timestamp.csv")
            val writer = BufferedWriter(
                OutputStreamWriter(FileOutputStream(file, false), Charsets.UTF_8),
                32768
            )
            val logger = Datalogger(file = file, writer = writer)
            if (!logger.writeRow(header)) {
                val error = logger.errorMessage
                logger.close()
                throw IOException("Unable to write CSV header: $error")
            }
            return logger
        }
    }
}