package org.firstinspires.ftc.teamcode.library.robot.systems.rgb

import com.qualcomm.robotcore.hardware.PWMOutput

class BlinkinController (private val blinkin : PWMOutput) {
    fun setColor(color:IndexableRGBPattern): Boolean =
            with(color.servoInput) {
                if (this in 1000..2000) {
                    blinkin.pulseWidthPeriod = this
                    return true
                }
                return false
            }
}