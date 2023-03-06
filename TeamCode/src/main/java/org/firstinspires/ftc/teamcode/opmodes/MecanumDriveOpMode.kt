package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.lib.LinearOpModeEx
import org.firstinspires.ftc.teamcode.subsytems.ClawLift
import org.firstinspires.ftc.teamcode.subsytems.Distances
import org.firstinspires.ftc.teamcode.subsytems.MecanumDrive
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign

class MecanumDriveOpMode : LinearOpModeEx() {

    @TeleOp
    class MecanumDrive : LinearOpMode() {
        override fun runOpMode() = MecanumDriveOpMode().runOpMode(this)
    }

    val clawLift by lazy { ClawLift(hardware) }
    val drive by lazy { MecanumDrive(hardware) }
    val distances by lazy { Distances(hardware) }

    override fun once() {
        clawLift.isOpen = true
    }

    override fun loop() {
        when {
            gamepad1.leftBumper.isDown && !gamepad1.rightBumper.isDown -> drive.drive(0.0, 0.0, -0.45)
            gamepad1.rightBumper.isDown && !gamepad1.leftBumper.isDown -> drive.drive(0.0, 0.0, 0.45)
            gamepad1.x.isDown -> drive.drive(0.0, 0.0, -0.25)
            gamepad1.b.isDown -> drive.drive(0.0, 0.0, 0.25)
            gamepad1.leftTrigger != 0.0 -> drive.drive(PI /2, gamepad1.leftTrigger * 0.5, 0.0)
            gamepad1.rightTrigger != 0.0 -> drive.drive(PI /2, gamepad1.rightTrigger * -0.5, 0.0)
            gamepad1.dpadUp.isDown -> drive.drive(PI /2, 0.5, 0.0)
            gamepad1.dpadLeft.isDown -> drive.drive(PI, 0.5, 0.0)
            gamepad1.dpadDown.isDown -> drive.drive(3* PI /2, 0.5, 0.0)
            gamepad1.dpadRight.isDown -> drive.drive(0.0, 0.5, 0.0)
            else -> {
                var speed = 1.0 - (if (gamepad1.leftBumper.isDown && gamepad1.rightBumper.isDown) 0.75 else 0.5)
                speed *= if (clawLift.height >= ClawLift.MEDIUM_JUNCTION) 0.8 else 1.0
                drive.drive(
//            gamepad1.leftStick.angle - gyro.robotHeading,
                    gamepad1.leftStick.angle,
                    gamepad1.leftStick.dist.pow(3) * speed,
                    gamepad1.rightStick.x * speed
                )
            }
        }

        clawLift.height = when {
            gamepad2.dpadDown.justPressed -> ClawLift.DOWN
            gamepad2.dpadRight.justPressed -> ClawLift.MEDIUM_JUNCTION
            gamepad2.dpadLeft.justPressed -> ClawLift.LOW_JUNCTION
            gamepad2.dpadUp.justPressed -> ClawLift.HIGH_JUNCTION
            gamepad2.a.justPressed -> ClawLift.TWO_CONES
            gamepad2.x.justPressed -> ClawLift.THREE_CONES
            gamepad2.b.justPressed -> ClawLift.FOUR_CONES
            gamepad2.y.justPressed -> ClawLift.FIVE_CONES
            else -> clawLift.height
        }
        clawLift.height += (gamepad2.leftStick.y * 8)
        telemetry.addData("lift", clawLift.height)

        when {
            gamepad2.a.justPressed ||
                    gamepad2.x.justPressed ||
                    gamepad2.b.justPressed ||
                    gamepad2.y.justPressed ||
                    gamepad2.dpadDown.justPressed -> clawLift.swivelPos = 0.0
            gamepad2.rightStick.x < -0.8 && clawLift.swivelPos == 0.0 -> clawLift.swivelPos = -1.0
            gamepad2.rightStick.x < -0.8 && clawLift.swivelPos == 1.0 -> clawLift.swivelPos = 0.0001
            gamepad2.rightStick.x > 0.8  && clawLift.swivelPos == 0.0 -> clawLift.swivelPos = 1.0
            gamepad2.rightStick.x > 0.8  && clawLift.swivelPos == -1.0 -> clawLift.swivelPos = 0.0001
            gamepad2.rightStick.y < -0.8 -> clawLift.swivelPos = 0.0
            clawLift.swivelPos.absoluteValue == 0.0001 && gamepad2.rightStick.x.absoluteValue < 0.8 -> clawLift.swivelPos = 0.0
        }

        if (gamepad2.rightBumper.justPressed) clawLift.toggle()

        clawLift.tick()


        telemetry.addData("wall left", distances.wallDistLeft())
        telemetry.addData("wall right", distances.wallDistRight())
        telemetry.addData("pole left", distances.poleDistLeft())
        telemetry.addData("pole right", distances.poleDistRight())
    }

    override fun finish() {
        clawLift.swivelPos = 0.0
    }
}