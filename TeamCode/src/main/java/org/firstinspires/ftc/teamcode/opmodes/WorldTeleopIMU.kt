package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeBuffer
import org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip
import org.firstinspires.ftc.teamcode.library.functions.toDegrees
import org.firstinspires.ftc.teamcode.library.functions.toRadians
import org.firstinspires.ftc.teamcode.library.functions.truncate
import org.firstinspires.ftc.teamcode.library.robot.systems.IMUDrivetrainControllerKotlin

@TeleOp(name = "World Teleop + IMU", group = "World")
class WorldTeleopIMU : WorldTeleop() {

    var useIMU = true
    lateinit var imuController: IMUDrivetrainControllerKotlin
    var zeroAngle = 0.0

    override fun init() {
        super.init()
        imuController = IMUDrivetrainControllerKotlin(robot.imuA, robot.imuB, robot.holonomic, telemetry)
    }

    override fun controlDrivetrain() {
        if (gamepad1.a or gamepad1.b) useIMU = false
        else if (gamepad1.x) useIMU = true

        if (useIMU) {
            telemetry.addData("Driving Mode", "IMU")
            if (gamepad1.x) {
                zeroAngle = imuController.headingA.toDouble().toRadians()
            }
            val xTarget =  gamepad1.left_stick_x.toDouble().processSpeed().processDirection().run { if (this == 0.0) 0.000001 else this }
            val yTarget = -gamepad1.left_stick_y.toDouble().processSpeed().processDirection()
            val r: Double
            var theta: Double
            val axisConversionAngle = Math.PI / 4
            val xPrime: Double
            val yPrime: Double

            // calculate r
            r = Math.sqrt(Math.pow(xTarget, 2.0) + Math.pow(yTarget, 2.0))
            // calculate theta
            theta = Math.atan(yTarget / xTarget) - Math.PI / 2
            if (xTarget < 0) theta += Math.PI
            theta += (zeroAngle - imuController.headingA.toDouble().toRadians())
            // calculate x and y prime
            xPrime = r * Math.cos(theta - axisConversionAngle)
            yPrime = r * Math.sin(theta - axisConversionAngle)
            // telemetry
            telemetry.addLine().addData("heading", imuController.headingA.toDouble().truncate(2))
            telemetry.addLine().addData("\"zero\" angle", zeroAngle.toDegrees().truncate(2))
            telemetry.addLine().addData("x input", xTarget.truncate(2)).addData("y input", yTarget.truncate(2))
            telemetry.addLine().addData("r", r.truncate(2))
            telemetry.addLine().addData("theta", theta.toDegrees().truncate(2))
            telemetry.addLine().addData("x prime", xPrime.truncate(2)).addData("y prime", yPrime.truncate(2))
            robot.holonomic.runWithoutEncoderPrime(xPrime, yPrime, gamepad1.right_stick_x.toDouble().processSpeed())
        } else super.controlDrivetrain()
    }
}