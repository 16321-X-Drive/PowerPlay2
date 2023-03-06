package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.atan2
import kotlin.math.hypot

class GamepadEx(private val gamepad: Gamepad) {

    enum class JoystickSide {
        LEFT,
        RIGHT
    }

    class Joystick(private val gamepad: Gamepad, private val side: JoystickSide) {

        val x: Double
            get() = if (side == JoystickSide.LEFT) gamepad.left_stick_x.toDouble() else gamepad.right_stick_x.toDouble()

        val y: Double
            get() = if (side == JoystickSide.LEFT) -gamepad.left_stick_y.toDouble() else -gamepad.right_stick_y.toDouble()

        val dist: Double
            get() = hypot(x, y)

        val angle: Double
            get() = atan2(y, x)
    }

    class Button {
        private var lastState = false

        val isDown: Boolean
            get() = lastState

        var justPressed: Boolean = false
            private set

        fun tick(currentBtnState: Boolean) {
            if (currentBtnState && !lastState) {
                lastState = true
                justPressed = true
            } else if (!currentBtnState) {
                lastState = false
                justPressed = false
            } else {
                justPressed = false
            }
        }
    }

    val leftStick = Joystick(gamepad, JoystickSide.LEFT)
    val rightStick = Joystick(gamepad, JoystickSide.RIGHT)

    val x = Button()
    val y = Button()
    val a = Button()
    val b = Button()

    val dpadLeft = Button()
    val dpadRight = Button()
    val dpadUp = Button()
    val dpadDown = Button()

    val leftBumper = Button()
    val rightBumper = Button()

    val leftTrigger: Double get() = gamepad.left_trigger.toDouble()
    val rightTrigger: Double get() = gamepad.right_trigger.toDouble()

    fun tick() {
        x.tick(gamepad.x)
        y.tick(gamepad.y)
        a.tick(gamepad.a)
        b.tick(gamepad.b)

        dpadLeft.tick(gamepad.dpad_left)
        dpadRight.tick(gamepad.dpad_right)
        dpadUp.tick(gamepad.dpad_up)
        dpadDown.tick(gamepad.dpad_down)

        leftBumper.tick(gamepad.left_bumper)
        rightBumper.tick(gamepad.right_bumper)
    }

}