package org.firstinspires.ftc.teamcode.subsytems

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.hardware.Hardware
import org.opencv.core.Rect
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

enum class Decision {
    Red, Green, Blue
}

class Detector(val hardware: Hardware, val rect: Rect) {

    private lateinit var pipeline: PowerPlayPipeline
    private lateinit var webcam: OpenCvWebcam

    fun readDecision() = pipeline.readDecision()

    fun init() {
        val cameraMonitorViewId = hardware.map.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardware.map.appContext.packageName
        )

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardware.map.get(
                WebcamName::class.java, "Webcam 1"
            ), cameraMonitorViewId
        )

        pipeline = PowerPlayPipeline(rect)

        webcam.openCameraDevice()
        webcam.setPipeline(pipeline)
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
    }

    fun close() {
        webcam.stopRecordingPipeline()
        webcam.stopStreaming()
        webcam.closeCameraDevice()
    }
}