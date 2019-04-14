package org.firstinspires.ftc.teamcode.library.robot.systems

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Velocity
import kotlin.math.abs

class IMUDrivetrainControllerKotlin(private val imuA: BNO055IMU,
        private val imuB : BNO055IMU,
        //    private Orientation imuBOriginalOrientation;
        private val drivetrain: Holonomic, private val telemetry: Telemetry) {
    //    private BNO055IMU imuB;

    private val imuAOriginalOrientation: Orientation? = null

    val headingA: Float
        get() = orientationA.firstAngle

    val rollA: Float
        get() = orientationA.secondAngle

    val pitchA: Float
        get() = orientationA.thirdAngle

    val orientationA: Orientation
        get() = imuA.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)

    val headingB: Float
        get() = orientationB.firstAngle

    val rollB: Float
        get() = orientationB.secondAngle

    val pitchB: Float
        get() = orientationB.thirdAngle

    val orientationB: Orientation
        get() = imuB.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)

    init {
        val initImu: ((BNO055IMU,Char)->Unit) = { imu: BNO055IMU, id:Char ->
            val parameters = BNO055IMU.Parameters()
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            parameters.calibrationDataFile = "IMU_${id}_CalibrationData.json" // see the calibration sample opmode
            parameters.loggingEnabled = true
            parameters.loggingTag = "IMU_${id}"
            parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
            imu.initialize(parameters)
            imu.startAccelerationIntegration(Position(), Velocity(), 1000)
        }
        initImu(imuA, 'A')
        initImu(imuB, 'B')
    }

    fun _rotateForDegrees(degrees: Float, power: Float) {
        var currentAngle = headingA
        val targetAngle = headingA + degrees

        var finalPower = (-power).toDouble()
        drivetrain.runWithoutEncoder(0.0, 0.0, finalPower)
        while (true) {
            currentAngle = orientationA.firstAngle
            if (currentAngle / targetAngle < 0.50) {
                finalPower = (power * (currentAngle / targetAngle)).toDouble()
                if (finalPower < 0.10 && finalPower >= 0) {
                    finalPower = -0.10
                } else if (finalPower > -0.10 && finalPower <= 0) {
                    finalPower = 0.10
                }
                drivetrain.runWithoutEncoder(0.0, 0.0, finalPower)
            }
            telemetry.addData("current", currentAngle)
            telemetry.addData("target", targetAngle)
            telemetry.update()
        }
    }

    fun rotateForDegrees(degrees: Float, power: Double) {
        val elapsedTime = ElapsedTime()
        elapsedTime.startTime()
        val targetHeading = with(headingA + degrees) {
            when {
                this > 180 -> this-360
                this < -180 -> this+360
                else -> this
            }
        }

        while (!(headingA in targetHeading-1..targetHeading+1)) {
            drivetrain.runWithoutEncoder(0.0,0.0, when {
                headingA > targetHeading -> power - 0.20 * abs(headingA / targetHeading)
                else -> -power + 0.20 * abs(headingA / targetHeading)
            } )
            telemetry.addData("Current Heading", headingA)
            telemetry.addData("Target Heading", targetHeading)
            telemetry.addData("Time", elapsedTime.seconds())
            telemetry.update()
        }
        while(!(headingA in targetHeading-0.5..targetHeading+0.5)) {
            drivetrain.runWithoutEncoder(0.0,0.0, when {
                headingA > targetHeading -> (power - 0.30 * abs(headingA / targetHeading)) * 0.50
                else -> (-power + 0.20 * abs(headingA / targetHeading)) * 0.50
            } )
            telemetry.addData("Current Heading", headingA)
            telemetry.addData("Target Heading", targetHeading)
            telemetry.addData("Time", elapsedTime.seconds())
            telemetry.update()
        }
        drivetrain.stop()
        elapsedTime.reset()
    }

    fun meetTarget(degrees: Float, power: Double) = rotateForDegrees(headingA - degrees, power)

    fun rotateSuper(degrees: Float) {
        val target = -degrees
        val time = ElapsedTime()
        time.reset()
        time.startTime()
        var temp: Float
        while (time.seconds() < 8) {
            temp = headingA

            if (target > 0 && temp < 0) {
                drivetrain.runWithoutEncoder(0.0, 0.0, .40 - time.seconds() * .05)
            } else if (target < 0 && temp > 0) {
                drivetrain.runWithoutEncoder(0.0, 0.0, -.40 + time.seconds() * .05)
            } else if (temp > target) {
                drivetrain.runWithoutEncoder(0.0, 0.0, .40 - time.seconds() * .05)
            } else if (temp < target) {
                drivetrain.runWithoutEncoder(0.0, 0.0, -.40 + time.seconds() * .05)
            }
        }
        drivetrain.stop()
    }
}
