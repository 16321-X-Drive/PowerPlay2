package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.AnalogSensor

private const val BASE: Double = 30.39
private const val SLOPE: Double = 219.6

// 20 cm = .22
// 30 cm = .26
// 40 cm = .31
// 50 cm = .355
// 60 cm = .4

class MaxbotixDistanceSensor(private val analogSensor: AnalogInput) {

    val distanceCm: Double
        get() = analogSensor.voltage * SLOPE - BASE

    val distanceIn: Double
        get() = distanceCm / 2.54

}