package org.firstinspires.ftc.teamcode.subsytems

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.hardware.Hardware

class Distances(hardware: Hardware) {
    private val longDistLeft = hardware.longDistLeft
    private val longDistRight = hardware.longDistRight
    private val poleDistLeft = hardware.poleDistLeft
    private val poleDistRight = hardware.poleDistRight

    fun wallDistLeft() = longDistLeft.distanceIn + 13.75 / 2.0
    fun wallDistRight() = longDistRight.distanceIn + 13.75 / 2.0

    fun poleDistLeft() = poleDistLeft.getDistance(DistanceUnit.INCH)
    fun poleDistRight() = poleDistRight.getDistance(DistanceUnit.INCH)
}