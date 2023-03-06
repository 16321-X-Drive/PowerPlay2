package org.firstinspires.ftc.teamcode.lib

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.*
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.Hardware

open class LinearOpModeEx {
    lateinit var opMode: LinearOpMode

    val gamepad1: GamepadEx by lazy { GamepadEx(opMode.gamepad1) }
    val gamepad2: GamepadEx by lazy { GamepadEx(opMode.gamepad2) }

    fun waitForStart() {
        opMode.waitForStart()
    }

    fun idle() {
        opMode.idle()
    }

    fun sleep(milliseconds: Long) {
        opMode.sleep(milliseconds)
    }

    val isActive: Boolean get() = opMode.opModeIsActive()
    val isInInit: Boolean get() = opMode.opModeInInit()
    val isStarted: Boolean get() = opMode.isStarted
    val isStopRequested: Boolean get() = opMode.isStopRequested

    val runtime: Double get() = opMode.runtime

    val telemetry: Telemetry get() = MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().telemetry)
    val hardwareMap: HardwareMap get() = opMode.hardwareMap

    val hardware by lazy { Hardware(hardwareMap) }

    fun runOpMode(opMode: LinearOpMode) {
        this.opMode = opMode

        init()

        waitForStart()

        once()

        while (isActive && !isAuto()) {
            gamepad1.tick()
            gamepad2.tick()
            loop()
            opMode.telemetry.update()
        }

        finish()
    }

    open fun isAuto() = false

    open fun init() {}

    open fun once() {}

    open fun loop() {}

    open fun finish() {}
}