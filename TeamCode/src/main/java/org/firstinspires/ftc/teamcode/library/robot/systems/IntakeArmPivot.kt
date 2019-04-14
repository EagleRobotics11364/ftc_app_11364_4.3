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
        val storedFullProportion = fullProportion
        motor.power =
                when (ignorePowerRegulation) {
                    true -> 0.5 * input
                    false ->
                        when {
                            input > 0.03 -> {
                                when {
                                    fullProportion > 0.90   -> 0.0
                                    else                    -> input * 0.5
                                }
                            }
                            input < -0.03 -> {
                                when {
                                    fullProportion < 0.05 -> 0.0
                                    else -> input * 0.5
                                }
                            }
                            else -> when {
                                fullProportion in 0.20..0.65    -> 0.20
                                else                     -> 0.00

                            }
// input > 0.03 ->
////                                if (potentiometer.voltage > MIDPOINT_VERTICAL_VOLTAGE) {
////                                    when {
////                                        upperProportion > 0.95  -> 0.0
////                                        else                    -> input * 0.4 * (1 - 0.9 * fullProportion)
////                                    }
////                                } else input
//                                when {
//                                    fullProportion > 0.90 -> 0.0
////                                    else -> input * 0.8 * (1 - 0.9 * fullProportion)
//                                    else -> ((55.556 * storedFullProportion.pow(6))
//                                            + (-177.56 * storedFullProportion.pow(5))
//                                            + (211 * storedFullProportion.pow(4))
//                                            + (-112.58 * storedFullProportion.pow(3))
//                                            + (24.982 * storedFullProportion.pow(2))
//                                            + (-2.0345 * storedFullProportion)
//                                            + (0.6433)) * input.absoluteValue
//                                }
//                            input < -0.03 -> ((58.644 * storedFullProportion.pow(6))
//                                    + (-225.03 * storedFullProportion.pow(5))
//                                    + (308.64 * storedFullProportion.pow(4))
//                                    + (-178.15 * storedFullProportion.pow(3))
//                                    + (34.445 * storedFullProportion.pow(2))
//                                    + (1.2954 * storedFullProportion)
//                                    + (-0.3187)) * input.absoluteValue
//
//                            else -> when {
//                                fullProportion in 0.20..0.65 -> 0.20
//
//                                else -> 0.00
//
//                            }
                        }


//                    false -> (                   (-12.411 * storedFullProportion.pow(6))
//                            +       ( 60.094 * storedFullProportion.pow(5))
//                            +       (-113.23  * storedFullProportion.pow(4))
//                            +       ( 103.75  * storedFullProportion.pow(3))
//                            +       (-45.947 * storedFullProportion.pow(2))
//                            +       ( 7.3317  * storedFullProportion)
//                            +       (-0.0349)) * input.absoluteValue
                }

    }

    override fun stop() {
        motor.power = 0.0
    }
}