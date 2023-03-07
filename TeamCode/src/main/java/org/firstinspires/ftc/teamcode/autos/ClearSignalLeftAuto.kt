package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.LinearOpModeEx
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.subsytems.ClawLift
import org.firstinspires.ftc.teamcode.subsytems.Decision
import org.firstinspires.ftc.teamcode.subsytems.Detector
import org.firstinspires.ftc.teamcode.subsytems.Distances
import org.opencv.core.Rect
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign

class ClearSignalLeftAuto : LinearOpModeEx() {
    @Autonomous(name = "Left 1+2", preselectTeleOp = PRESELECTED_TELEOP)
    class ClearSignal : LinearOpMode() {
        override fun runOpMode() = ClearSignalLeftAuto().runOpMode(this)
    }

    override fun isAuto() = true

    val drive: SampleMecanumDrive by lazy { SampleMecanumDrive(hardwareMap) }
    val clawLift: ClawLift by lazy { ClawLift(hardware) }
    val distances: Distances by lazy { Distances(hardware) }
    val detector: Detector by lazy { Detector(hardware, Rect(140, 150, 30, 15)) }

    fun swivelCenter() {
        clawLift.swivelPos = 0.0
        sleep(250)
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
        -34.0, -72 + 13.5 / 2, PI / 2
    )
    val HEIGHT = -12.0
    lateinit var traj1: TrajectorySequence

    override fun init() {
        detector.init()

        traj1 = drive.trajectorySequenceBuilder(START)
            .splineToConstantHeading(Vector2d(-34.0, -41.0), PI / 2)
            .splineToLinearHeading(Pose2d(-50.0, -39.0, PI * 3 / 5), PI).forward(3.0).back(6.0)
            .addDisplacementMarker {
                clawLift.height = ClawLift.MEDIUM_JUNCTION
            }
            .lineToLinearHeading(Pose2d(-34.0, -32.0, PI / 2)).build()


        clawLift.swivelPos = 1.0
        clawLift.isOpen = false
    }

    fun cycle(stackHeight: Double, poleHeight: Double, stack: Pose2d, pole: Pose2d) {
        clawLift.height = stackHeight
        turnToHeading(PI)

        val traj3 = drive.trajectoryBuilder(drive.poseEstimate)
            .lineToLinearHeading(stack)
            .build()

        drive.followTrajectory(traj3)

        grab()
        sleep(500)
        clawLift.height = poleHeight
        sleep(500)

        val traj4 = drive.trajectoryBuilder(drive.poseEstimate)
            .lineToLinearHeading(pole)
            .build()

        drive.followTrajectory(traj4)
        moveToPole(-0.3, { distances.poleDistLeft() }, 0)
        clawLift.swivelPos = -1.0
        sleep(500)
        drive.followTrajectory(
            drive.trajectoryBuilder(drive.poseEstimate)
                .lineToLinearHeading(drive.poseEstimate.plus(Pose2d(-0.5, -2.0)))
                .build()
        )

        lower()
        drop()
        swivelCenter()

    }

    override fun once() {
        drive.poseEstimate = START

        val decision = detector.readDecision()
        telemetry.addData("decision", decision)
        telemetry.update()

        drive.followTrajectorySequence(traj1)

        clawLift.tick()
        moveToPole(0.3, { distances.poleDistRight() })
        drive.followTrajectory(
            drive.trajectoryBuilder(drive.poseEstimate)
                .lineToLinearHeading(drive.poseEstimate.plus(Pose2d(-1.0, 0.0)))
                .build()
        )
        lower()
        drop()
        swivelCenter()
        clawLift.height = ClawLift.FIVE_CONES

        val traj2 = drive.trajectoryBuilder(drive.poseEstimate)
            .lineToLinearHeading(Pose2d(START.x, HEIGHT, PI / 2))
            .build()

        drive.followTrajectory(traj2)
        turnToHeading(PI)

        var yEstimate = -72 + distances.wallDistLeft()
        telemetry.addData("y est", yEstimate)
        telemetry.update()
        if (yEstimate !in -20.0..-10.0) yEstimate = drive.poseEstimate.y
        drive.poseEstimate = drive.poseEstimate.copy(y = yEstimate)

        val stack1 = Pose2d(-59.25, HEIGHT, PI)
        val stack2 = Pose2d(-60.0, HEIGHT, PI)
        val mediumPole = Pose2d(-30.0, HEIGHT - 1.5, PI)
        val lowPole = Pose2d(-54.0, HEIGHT - 1.5, PI)

        if (decision == Decision.Green) {
            cycle(ClawLift.FIVE_CONES, ClawLift.LOW_JUNCTION, stack1, lowPole)
            cycle(ClawLift.FOUR_CONES, ClawLift.MEDIUM_JUNCTION, stack2, mediumPole)
        } else {
            cycle(ClawLift.FIVE_CONES, ClawLift.MEDIUM_JUNCTION, stack1, mediumPole)
            cycle(ClawLift.FOUR_CONES, ClawLift.LOW_JUNCTION, stack2, lowPole)
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