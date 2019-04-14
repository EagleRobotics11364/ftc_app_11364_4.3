package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.library.robot.BaseRobot
import org.firstinspires.ftc.teamcode.library.robot.systems.IMUDrivetrainController
import org.firstinspires.ftc.teamcode.library.robot.systems.IMUDrivetrainControllerKotlin

import org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeBuffer
import org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip

//@TeleOp(name = "State TeleOp + IMU", group = "LT")
class StateTeleopIMU : OpMode() {
    lateinit var robot: BaseRobot

    private var slow = false
    private var reverse = false
    private lateinit var imuController: IMUDrivetrainControllerKotlin
    private var zeroAngle: Double = 0.0

    override fun init() {
        robot = BaseRobot(hardwareMap)
        imuController = IMUDrivetrainControllerKotlin(robot.imuA, robot.imuB, robot.holonomic, telemetry)
    }

    override fun loop() {
        if (gamepad1.dpad_up)
            slow = false
        else if (gamepad1.dpad_down) slow = true
        if (gamepad1.x) {
            zeroAngle = imuController.headingA.toDouble().toRadians()
        }

        var xTarget = gamepad1.left_stick_x.toDouble()
        var yTarget = -gamepad1.left_stick_y.toDouble()
        val r: Double
        var theta: Double
        val axisConversionAngle = Math.PI / 4
        val xPrime: Double
        val yPrime: Double
        val xPower: Double
        val yPower: Double


        // set motors mode

        // calculate r
        r = Math.sqrt(Math.pow(xTarget, 2.0) + Math.pow(yTarget, 2.0))
        // calculate theta
        if (xTarget == 0.0) xTarget = 0.00001
        theta = Math.atan(yTarget / xTarget) - Math.PI / 2
        if (xTarget < 0) theta += Math.PI
        theta += (zeroAngle - imuController.headingA.toDouble().toRadians())
        // calculate x and y prime
        xPrime = r * Math.cos(theta - axisConversionAngle)
        yPrime = r * Math.sin(theta - axisConversionAngle)

        telemetry.addLine().addData("heading", imuController.headingA)
        telemetry.addLine().addData("\"zero\" angle", zeroAngle.toDegrees())
        telemetry.addLine().addData("x input", xTarget).addData("y input", yTarget)
        telemetry.addLine().addData("r", r)
        telemetry.addLine().addData("theta", theta.toDegrees())
        telemetry.addLine().addData("x prime", xPrime).addData("y prime", yPrime)
//        telemetry.addLine().addData("x power", xPower).addData("y power", yPower)
        telemetry.update()
        robot.holonomic.runWithoutEncoderPrime(xPrime, yPrime, gamepad1.right_stick_x.toDouble())


    }
    private fun Double.toDegrees() = this * 180 / Math.PI
    private fun Double.toRadians() = this * Math.PI / 180
}
