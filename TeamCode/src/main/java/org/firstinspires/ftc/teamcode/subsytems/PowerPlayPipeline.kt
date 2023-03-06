package org.firstinspires.ftc.teamcode.subsytems

import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class PowerPlayPipeline(private val area: Rect) : OpenCvPipeline() {

    companion object {
        val RED = Scalar(255.0, 0.0, 0.0)
        val GREEN = Scalar(0.0, 255.0, 0.0)
        val BLUE = Scalar(0.0, 0.0, 255.0)

        fun averageHues(hues: Mat): Double {
            var xTot = 0.0
            var yTot = 0.0
            for (x in 0 until hues.width()) {
                for (y in 0 until hues.height()) {
                    val cell = hues.get(y, x)[0]
                    val angle = Math.toRadians(cell)
                    xTot += cos(angle)
                    yTot += sin(angle)
                }
            }
            val count = hues.width() * hues.height()
            val xAvg = xTot / count
            val yAvg = yTot / count
            val angle = atan2(yAvg, xAvg)
            return Math.toDegrees(angle) * 2
        }
    }

    private var hls = Mat()
    private var hue = Mat()
    private lateinit var hueRegion: Mat
    var decision: Decision = Decision.Blue
        private set

    private fun extractHue(input: Mat) {
        Imgproc.cvtColor(input, hls, Imgproc.COLOR_RGB2HLS)
        Core.extractChannel(hls, hue, 0)
    }

    override fun init(firstFrame: Mat) {
        extractHue(firstFrame)
        hueRegion = hue.submat(area)
    }

    override fun processFrame(input: Mat): Mat {
        extractHue(input)
        decision = readDecision()

        Imgproc.rectangle(
            input,
            area,
            when (decision) {
                Decision.Red -> RED
                Decision.Green -> GREEN
                Decision.Blue -> BLUE
            },
            2
        )

        return input
    }

    fun readDecision(): Decision =
        when (averageHues(hueRegion)) {
            in 0.0..60.0 -> Decision.Red
            in 60.0..170.0 -> Decision.Green
            in 170.0..295.0 -> Decision.Blue
            else -> Decision.Red
        }

}