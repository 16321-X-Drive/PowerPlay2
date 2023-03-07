package org.firstinspires.ftc.teamcode.subsytems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.Hardware
import kotlin.math.absoluteValue
import kotlin.math.sign

class ClawLift(hardware: Hardware) {

    companion object {
        var CLAW_OPEN = 0.25
        var CLAW_CLOSED = 0.4

        var SWIVEL_CENTER = 0.48
        var SWIVEL_RIGHT_OFFSET = -0.33

        var DOWN = 0.0
        var TWO_CONES = 150.0 - 20.0
        var THREE_CONES = 350.0 - 20.0
        var FOUR_CONES = 500.0 - 20.0
        var FIVE_CONES = 650.0 - 20.0
        var LOW_JUNCTION = 1800.0
        var MEDIUM_JUNCTION = 3000.0
        var HIGH_JUNCTION = 4200.0
        var LIFT_RANGE = 0..4500
    }

    private val leftMotor = hardware.leftLift
    private val rightMotor = hardware.rightLift
    val reset = hardware.resetLift

    private val swivel: Servo = hardware.swivel

    private val claw: Servo = hardware.claw

    var height = 0.0
        set(value) {
            field = value
            leftMotor.targetPosition = field.toInt().coerceIn(LIFT_RANGE)
            rightMotor.targetPosition = field.toInt().coerceIn(LIFT_RANGE)
        }
    private var resetting = false

    fun Double.constrainMagnitude(max: Double) = this.sign * this.absoluteValue.coerceAtMost(max)

    fun constrainSwivelPos(desired: Double): Double =
        when (leftMotor.currentPosition.coerceAtMost(height.toInt())) {
            in -100..700 -> desired.constrainMagnitude(0.15)
            in 700..1400 -> desired.constrainMagnitude(0.15 + (leftMotor.currentPosition.toDouble() - 700.0) * 0.85 / 700.0)
            else -> desired.constrainMagnitude(1.0)
        }

    var swivelPos: Double = 0.0
        set(value) {
            field = value
            swivel.position = SWIVEL_CENTER + constrainSwivelPos(field) * SWIVEL_RIGHT_OFFSET
        }

    var isOpen = false
        set(value) {
            field = value
            claw.position = if (field) CLAW_OPEN else CLAW_CLOSED
        }

    init {
        rightMotor.direction = DcMotorSimple.Direction.REVERSE

        leftMotor.power = 1.0
        rightMotor.power = 1.0
        leftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        height = DOWN
        leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun toggle() {
        isOpen = !isOpen
    }

    fun tick() {
        if (!resetting) {
            leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        if (reset.isPressed) {
            leftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            rightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            leftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            rightMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            resetting = false
        }

        // Readjust max position
        swivelPos = swivelPos
    }

    fun reset() {
        height = DOWN
        resetting = true
        leftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftMotor.power = -1.0
        rightMotor.power = -1.0
    }
}