package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor

class Hardware(hm: HardwareMap) {

    val map = hm

    val leftFront = hm.get(DcMotorEx::class.java, "leftFront")!!
    val leftBack = hm.get(DcMotorEx::class.java, "leftBack")!!
    val rightFront = hm.get(DcMotorEx::class.java, "rightFront")!!
    val rightBack = hm.get(DcMotorEx::class.java, "rightBack")!!

    val swivel = hm.get(Servo::class.java, "swivel")!!
    val claw = hm.get(Servo::class.java, "claw")!!

    val leftLift = hm.get(DcMotorEx::class.java, "leftLift")!!
    val rightLift = hm.get(DcMotorEx::class.java, "rightLift")!!
    val resetLift = hm.get(TouchSensor::class.java, "liftReset")!!

    val longDistLeft = MaxbotixDistanceSensor(hm.get(AnalogInput::class.java, "longDistLeft")!!)
    val longDistRight = MaxbotixDistanceSensor(hm.get(AnalogInput::class.java, "longDistRight")!!)

    val poleDistLeft = hm.get(DistanceSensor::class.java, "poleDistLeft")!!
    val poleDistRight = hm.get(DistanceSensor::class.java, "poleDistRight")!!

    val imu = hm.get(BNO055IMU::class.java, "imu")!!
}
