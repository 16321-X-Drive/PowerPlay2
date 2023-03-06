package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.lib.LinearOpModeEx
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsytems.ClawLift
import org.firstinspires.ftc.teamcode.subsytems.Decision
import org.firstinspires.ftc.teamcode.subsytems.Detector
import org.firstinspires.ftc.teamcode.subsytems.Distances
import org.opencv.core.Rect
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign

class LeftAuto : LinearOpModeEx() {
    @Autonomous(preselectTeleOp = PRESELECTED_TELEOP)
    class Left : LinearOpMode() {
        override fun runOpMode() = LeftAuto().runOpMode(this)
    }

    override fun isAuto() = true

    val drive: SampleMecanumDrive by lazy { SampleMecanumDrive(hardwareMap) }
    val clawLift: ClawLift by lazy { ClawLift(hardware) }
    val distances: Distances by lazy { Distances(hardware) }
    val detector: Detector by lazy { Detector(hardware, Rect(140, 150, 30, 15)) }

    fun swivelLeft() {
        clawLift.swivelPos = -1.0
        sleep(750)
    }

    fun swivelRight() {
        clawLift.swivelPos = 1.0
        sleep(750)
    }

    fun swivelCenter() {
        clawLift.swivelPos = 0.0
        sleep(750)
    }

    fun lower() {
        clawLift.height -= 200
        sleep(500)
    }

    fun drop() {
        clawLift.isOpen = true
        sleep(500)
    }

    fun grab() {
        clawLift.isOpen = false
        sleep(500)
    }

    fun moveToPole(power: Double, dist: () -> Double, reverseTime: Long = 0) {
        drive.setMotorPowers(power, power, power, power)
        while (dist() > 15) {
            drive.update()
        }
        drive.setMotorPowers(-power, -power, -power, -power)
        val start = System.currentTimeMillis()
        while (System.currentTimeMillis() - start < reverseTime) {
            drive.update()
        }
        drive.setMotorPowers(0.0, 0.0, 0.0, 0.0)
    }

    fun Double.coerceMagMax(max: Double) = this.sign * this.absoluteValue.coerceAtMost(max)
    fun Double.coerceMagMin(min: Double) = this.sign * this.absoluteValue.coerceAtLeast(min)

    fun turnToHeading(targetAngle: Double, tolerance: Double = 0.005) {
        while (!isStopRequested) {
            val headingError = drive.poseEstimate.heading - targetAngle
            if (headingError < tolerance && headingError > -tolerance) break
            val power = (headingError / 3.0).coerceMagMax(0.5).coerceMagMin(0.15)
            drive.setMotorPowers(power, power, -power, -power)
            drive.update()
        }
        drive.setMotorPowers(0.0, 0.0, 0.0, 0.0)
    }

    val START = Pose2d(
        -34.0,
        -72 + 13.5 / 2,
        PI / 2
    )
    val HEIGHT = -12.0
    lateinit var traj1: Trajectory

    override fun init() {
        detector.init()

        clawLift.isOpen = false
        clawLift.swivelPos = 0.0

        traj1 = drive.trajectoryBuilder(START)
            .forward(38.0)
            .build()
    }

    override fun once() {
        drive.poseEstimate = START

        val decision = detector.readDecision()
        telemetry.addData("decision", decision)
        telemetry.update()

        clawLift.height = ClawLift.MEDIUM_JUNCTION
        drive.followTrajectory(traj1)
        moveToPole(0.3, { distances.poleDistRight() })
        swivelRight()
        lower()
        drop()
        swivelCenter()
        clawLift.height = ClawLift.FIVE_CONES + 300

        val traj2 = drive.trajectorySequenceBuilder(drive.poseEstimate)
            .forward(20.0)
            .lineToLinearHeading(Pose2d(START.x, HEIGHT, PI / 2))
            .build()

        drive.followTrajectorySequence(traj2)
        turnToHeading(PI)

        val stack = Pose2d(-58.5, HEIGHT, PI)

        var yEstimate = -72 + distances.wallDistLeft()
        if (yEstimate !in -20.0..-10.0) yEstimate = drive.poseEstimate.y
        drive.poseEstimate = drive.poseEstimate.copy(y = yEstimate)

        val heights = arrayOf(
            ClawLift.FIVE_CONES,
            ClawLift.FOUR_CONES,
            ClawLift.THREE_CONES,
            ClawLift.TWO_CONES,
            ClawLift.DOWN
        )
        for (height in 0 until 2) {
            clawLift.height = heights[height]
            turnToHeading(PI)

            val thisStack = stack.copy(x = stack.x - height * 1.5)
            val traj3 = drive.trajectoryBuilder(drive.poseEstimate)
                .lineToLinearHeading(thisStack)
                .build()

            drive.followTrajectory(traj3)

            grab()
            sleep(500)
            clawLift.height = ClawLift.MEDIUM_JUNCTION
            sleep(500)

            val traj4 = drive.trajectoryBuilder(drive.poseEstimate)
                .lineToLinearHeading(Pose2d(-30.0, HEIGHT - 1.5, PI))
                .build()

            drive.followTrajectory(traj4)
            moveToPole(-0.3, { distances.poleDistLeft() }, 0)
            clawLift.swivelPos = -1.0
            sleep(500)
            drive.followTrajectory(
                drive.trajectoryBuilder(drive.poseEstimate)
                    .lineToLinearHeading(drive.poseEstimate.plus(Pose2d(-0.5, -0.5)))
                    .build()
            )

            lower()
            drop()
            swivelCenter()
        }

        val park = when (decision) {
            Decision.Red -> Vector2d(-58.0, HEIGHT)
            Decision.Blue -> Vector2d(-36.0, HEIGHT)
            Decision.Green -> Vector2d(-12.0, HEIGHT)
        }
        val traj5 = drive.trajectoryBuilder(drive.poseEstimate)
            .lineToConstantHeading(park)
            .build()
        clawLift.height = 0.0
        drive.followTrajectory(traj5)

        turnToHeading(PI)
    }
}