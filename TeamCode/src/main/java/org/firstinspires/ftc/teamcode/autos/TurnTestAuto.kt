package org.firstinspires.ftc.teamcode.autos

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.LinearOpModeEx
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsytems.ClawLift
import org.firstinspires.ftc.teamcode.subsytems.Distances
import kotlin.math.PI

class TurnTestAuto : LinearOpModeEx() {
    @Autonomous(preselectTeleOp = PRESELECTED_TELEOP)
    class MyTurnTest : LinearOpMode() {
        override fun runOpMode() = TurnTestAuto().runOpMode(this)
    }

    override fun isAuto() = true

    val drive: SampleMecanumDrive by lazy { SampleMecanumDrive(hardwareMap) }

    override fun once() {
        for (i in 0..5) {
            val traj1 = drive.trajectorySequenceBuilder(Pose2d())
                .turn(PI/2)
                .build()

            drive.followTrajectorySequence(traj1)
        }
    }
}