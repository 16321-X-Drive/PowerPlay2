package org.firstinspires.ftc.teamcode.subsytems

import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.hardware.Hardware
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class MecanumDrive(hardware: Hardware) {

    private val leftFront = hardware.leftFront
    private val leftBack = hardware.leftBack
    private val rightFront = hardware.rightFront
    private val rightBack = hardware.rightBack

    init {
        configureMotors()
    }

    fun configureMotors() {
        rightFront.direction = DcMotorSimple.Direction.REVERSE
        rightBack.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.FORWARD
        leftBack.direction = DcMotorSimple.Direction.FORWARD
    }

    fun drive(theta: Double, power: Double, turn: Double) {
        // Taken from: https://www.youtube.com/watch?v=gnSW2QpkGXQ

        val sin = sin(theta - PI / 4 + PI)
        val cos = cos(theta - PI / 4 + PI)
        val max = abs(sin).coerceAtLeast(abs(cos))

        var leftFrontPow = power * cos / max - turn
        var rightFrontPow = power * sin / max + turn
        var leftBackPow = power * sin / max - turn
        var rightBackPow = power * cos / max + turn

        if (power + abs(turn) > 1) {
            leftFrontPow /= power + turn
            rightFrontPow /= power + turn
            leftBackPow /= power + turn
            rightBackPow /= power + turn
        }

        leftFront.power = leftFrontPow
        rightFront.power = rightFrontPow
        leftBack.power = leftBackPow
        rightBack.power = rightBackPow
    }
}
