package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.library.functions.MathOperations
import org.firstinspires.ftc.teamcode.library.functions.MusicPlayer
import org.firstinspires.ftc.teamcode.library.functions.truncate
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot

@TeleOp(name = "World TeleOp", group = "World")
open class WorldTeleop : OpMode() {
    lateinit var robot: BaseRobot
    lateinit var musicPlayer: MusicPlayer
    var musicButtonPressed = false
    var ledButtonPressed = false
    var currentLEDPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK
    var slow = false
    var reverse = false
    var gamepad1StickButtonPressed = false
    var doConstantPower = true
    var musicDiscovered = false
    override fun init() {
        robot = BaseRobot(hardwareMap)
        musicPlayer = MusicPlayer(hardwareMap)
        robot.outputDirectionSwitcherServo.position = BaseRobot.INTAKE_SILVER_OUTPUT_LEFT
    }

    override fun loop() {
        adjustDrivetrainSpeed()
        controlDrivetrain()
        controlAccessories()
        controlMusic()
        controlLEDs()
        telemetry.update()
    }

    override fun stop() {
        musicPlayer.stop()
    }

    fun adjustDrivetrainSpeed() {
        if (gamepad1.dpad_up) slow = false
        else if (gamepad1.dpad_down) slow = true

        if (gamepad1.left_stick_button) {
            if (!gamepad1StickButtonPressed) {
                gamepad1StickButtonPressed = true
                slow = !slow
            }
        } else
            gamepad1StickButtonPressed = false
        telemetry.addData("Speed", if (slow) "Slow" else "Normal")
    }

    open fun controlDrivetrain() {

        val x = -gamepad1.left_stick_y.toDouble().rangeClipAndBuffer().processSpeed().processDirection()
        val y = -gamepad1.left_stick_x.toDouble().rangeClipAndBuffer().processSpeed().processDirection()
        val z = gamepad1.right_stick_x.toDouble().rangeClipAndBuffer().processSpeed()
        robot.holonomic.runWithoutEncoder(x, y, z)

//        val directions = doubleArrayOf((-gamepad1.left_stick_y).toDouble(), (-gamepad1.left_stick_x).toDouble(), gamepad1.right_stick_x.toDouble())
//        for (i in directions.indices) {
//            directions[i] = MathOperations.rangeClip(directions[i], -1.0, 1.0)
//            directions[i] = MathOperations.rangeBuffer(directions[i], -0.1, 0.1, 0.0)
//            directions[i] *= if (gamepad1.left_bumper and gamepad1.right_bumper) 0.50 else 1.0 // if slow is true, multiply by 0.25
//        }
//        robot.holonomic.runWithoutEncoder((if (reverse) -1 else 1) * directions[0], (if (reverse) -1 else 1) * directions[1], directions[2])

        if (gamepad1.a)
            reverse = false
        else if (gamepad1.b) reverse = true
        telemetry.addData("Driving Mode", if (reverse) "Reverse" else "Normal")
    }

    fun controlAccessories() {
        if (gamepad2.right_bumper) {
            // control hanging screw
            robot.hangingScrew.manualControl(gamepad2.left_stick_y.toDouble())

            // disable intake motors
            robot.intakeArmExtensionMotor.power = 0.0
            robot.intakeArmPivotController.stop()
            robot.collectorServo.position = 0.5

        } else
        // stop hanging screw
            robot.hangingScrew.stop()

        // intake arm raise/lower (with auto override)
        robot.intakeArmPivotController.controlFromGamepadInput((gamepad2.left_stick_y).toDouble(), gamepad2.left_bumper)

        // intake arm extend/retract
        if (gamepad2.right_bumper) when {
            gamepad2.dpad_up -> doConstantPower = true
            gamepad2.dpad_down -> doConstantPower = false
        }
//        when (doConstantPower) {
//            true  -> {
//                if (robot.intakeArmPivotController.fullProportion >= 0.55 )robot.intakeArmExtensionMotor.power = 0.25 - (-gamepad2.right_stick_y) * 0.20
//                else robot.intakeArmExtensionMotor.power = (-gamepad2.right_stick_y).toDouble()
//            }
//            false -> robot.intakeArmExtensionMotor.power = (-gamepad2.right_stick_y).toDouble()
//        }
        if (!(gamepad2.right_stick_y in -0.03..0.03)) {
            robot.intakeArmExtensionMotor.power = -gamepad2.right_stick_y.toDouble()
        } else {
            robot.intakeArmExtensionMotor.power = when (doConstantPower) {
                true -> if (robot.intakeArmPivotController.fullProportion < 0.5) 0.00 else 0.20
                false -> 0.0
            }
        }
        // mineral box pivot
        if (gamepad2.a)
            robot.collectionBoxPivotServo.position = BaseRobot.INTAKE_BOX_POSITION_HOLDING
        else if (gamepad2.b)
            robot.collectionBoxPivotServo.position = BaseRobot.INTAKE_BOX_POSITION_COLLECTION_OR_DEPOSIT
        else if (gamepad2.left_stick_button) {
            robot.collectionBoxPivotServo.position =
                    when {
                        robot.intakeArmPivotController.fullProportion in 0.6..2.0 ->
                            BaseRobot.INTAKE_BOX_POSITION_COLLECTION_OR_DEPOSIT
                        else ->
                            BaseRobot.INTAKE_BOX_POSITION_HOLDING
                    }
        }

        // change output direction
        if (gamepad2.dpad_left)
            robot.outputDirectionSwitcherServo.position = BaseRobot.INTAKE_SILVER_OUTPUT_LEFT
        else if (gamepad2.dpad_right) robot.outputDirectionSwitcherServo.position = BaseRobot.INTAKE_SILVER_OUTPUT_RIGHT
        else if (gamepad2.dpad_down) robot.outputDirectionSwitcherServo.position = BaseRobot.INTAKE_SILVER_OUTPUT_CENTER

        // mineral collector speed
        robot.collectorServo.position =
                when {
                    gamepad2.right_trigger > 0.03 ->
                        0.50 + (gamepad2.right_trigger * 0.50)
                    gamepad2.left_trigger > 0.03 ->
                        0.50 - (gamepad2.left_trigger * 0.5)
                    gamepad1.right_trigger > 0.03 ->
                        0.50 + (gamepad1.right_trigger * 0.50)
                    gamepad1.left_trigger > 0.03 ->
                        0.50 - (gamepad1.left_trigger * 0.5)
                    else ->
                        0.50
                }

        // yeet the team marker
        if (gamepad2.y)
            robot.teamMarkerServo.position = BaseRobot.TEAM_MARKER_RELEASED // y is for yeet
        else if (gamepad2.x) robot.teamMarkerServo.position = BaseRobot.TEAM_MARKER_RETRACTED

        telemetry.addLine()
        telemetry.addLine("--Intake--")
        telemetry.addLine().addData("Voltage", robot.intakeArmPivotController.currentVoltage.truncate(2)).addData("Motor Power", robot.intakeArmPivotController.currentMotorPower.truncate(2))
        telemetry.addLine().addData("Lower %", robot.intakeArmPivotController.lowerProportion.truncate(2)).addData("Upper %", robot.intakeArmPivotController.upperProportion.truncate(2))
        telemetry.addLine().addData("Full %", robot.intakeArmPivotController.fullProportion.truncate(2))
        telemetry.addLine().addData("Arm Hold", doConstantPower).addData("Not Going Down", robot.intakeArmPivotController.notGoingDown)
        telemetry.addLine()
    }

    fun controlMusic() {
        if (gamepad1.right_stick_button or gamepad2.right_stick_button) {
            if (!musicButtonPressed) {
                musicDiscovered = true
                when (musicPlayer.isPlaying()) {
                    true -> {
                        musicPlayer.pause()
                        musicButtonPressed = true
                    }
                    false -> if (gamepad1.right_stick_button and gamepad2.right_stick_button) {
                        musicPlayer.play()
                        musicButtonPressed = true
                    }
                }

            }

        } else {
            musicButtonPressed = false
        }
        if (musicDiscovered) {
            telemetry.addData("Music", if (musicPlayer.isPlaying()) "Playing" else "Paused")
        }
    }

    fun controlLEDs() {
        if (gamepad1.y) {
            if (!ledButtonPressed) {
                if (ledOrder.isNotEmpty()) {
                    if (ledOrder.size - 1 > currentLEDSetting) currentLEDSetting++
                    else currentLEDSetting = 0
                    ledButtonPressed = true
                    robot.blinkin.setPattern(ledOrder[currentLEDSetting])
                }

            }
        } else ledButtonPressed = false
        telemetry.addData("LEDs", ledOrder[currentLEDSetting])
    }

    protected fun Double.processSpeed(): Double = if (slow) this / 2 else this
    protected fun Double.processDirection(): Double = if (reverse) -this else this
    protected fun Double.rangeClipAndBuffer(): Double {
        return MathOperations.rangeBuffer(
                MathOperations.rangeClip(this, -1.0, 1.0),
                -0.05, 0.05)
    }

    private val ledOrder = listOf(
            RevBlinkinLedDriver.BlinkinPattern.BLACK,
            RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE,
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE,
            RevBlinkinLedDriver.BlinkinPattern.VIOLET
    )
    private var currentLEDSetting = 0
}
