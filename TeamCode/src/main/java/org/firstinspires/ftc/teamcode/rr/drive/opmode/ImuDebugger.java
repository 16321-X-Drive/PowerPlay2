package org.firstinspires.ftc.teamcode.rr.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;

@Disabled
@TeleOp(group = "drive")
public class ImuDebugger extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.6,
                            -gamepad1.left_stick_x * 0.6,
                            -gamepad1.right_stick_x * 0.6
                    )
            );

            drive.update();

            telemetry.addData("x", drive.imu.getAngularOrientation().firstAngle);
            telemetry.addData("y", drive.imu.getAngularOrientation().secondAngle);
            telemetry.addData("z", drive.imu.getAngularOrientation().thirdAngle);
            telemetry.addData("xRotVel", drive.imu.getAngularVelocity().xRotationRate);
            telemetry.addData("yRotVel", drive.imu.getAngularVelocity().yRotationRate);
            telemetry.addData("zRotVel", drive.imu.getAngularVelocity().zRotationRate);
            telemetry.update();
        }
    }
}
