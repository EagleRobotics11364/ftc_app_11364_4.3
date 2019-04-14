package org.firstinspires.ftc.teamcode.library.robot.systems

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.absoluteValue
import kotlin.math.pow


class IntakeArmPivot(private val motor: DcMotor,
                     private val potentiometer: AnalogInput) : Stoppable {
    val ENDPOINT_LOWER_VOLTAGE: Double = 0.18
    val MIDPOINT_VERTICAL_VOLTAGE: Double = 0.55
    val ENDPOINT_UPPER_VOLTAGE: Double = 1.097
    var notGoingDown = true
    val currentVoltage: Double
        get() = potentiometer.voltage
    val upperProportion: Double
        get() =
            (potentiometer.voltage - MIDPOINT_VERTICAL_VOLTAGE) /
                    (ENDPOINT_UPPER_VOLTAGE - MIDPOINT_VERTICAL_VOLTAGE)
    val lowerProportion: Double
        get() =
            (potentiometer.voltage - ENDPOINT_LOWER_VOLTAGE) /
                    (MIDPOINT_VERTICAL_VOLTAGE - ENDPOINT_LOWER_VOLTAGE)

    val fullProportion: Double
        get() =
            (potentiometer.voltage - ENDPOINT_LOWER_VOLTAGE) /
                    (ENDPOINT_UPPER_VOLTAGE - ENDPOINT_LOWER_VOLTAGE)

    val currentMotorPower: Double
        get() = motor.power

    fun controlFromGamepadInput(input: Double, ignorePowerRegulation: Boolean) {
        motor.power =
                when (ignorePowerRegulation) {
                    true -> 0.5 * input
                    false ->
                        when {
                            input > 0.03 -> {
                                when {
                                    fullProportion > 0.90 -> 0.0
                                    else -> input * 0.5
                                }
                            }
                            input < -0.03 -> {
                                when {
                                    fullProportion < 0.05 -> 0.0
                                    else -> input * 0.5
                                }
                            }
                            else -> when {
                                fullProportion in 0.20..0.65 -> 0.20
                                else -> 0.00

                            }
                        }
                }
    }

    override fun stop() {
        motor.power = 0.0
    }
}