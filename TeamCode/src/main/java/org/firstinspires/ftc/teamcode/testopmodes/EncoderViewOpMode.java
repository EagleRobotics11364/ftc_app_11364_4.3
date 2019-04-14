package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;
@TeleOp(name="Encoder View OpMode", group="Test")
@Disabled
public class EncoderViewOpMode extends OpMode {
    BaseRobot robot;

    @Override
    public void init() {
        robot = new BaseRobot(hardwareMap);
        robot.holonomic.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double power = gamepad1.left_stick_x;
        String motor = "n/a";
        if (gamepad1.a) {
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(power);
            motor = "back right";
        } else if (gamepad1.b) {
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(power);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);
            motor = "front right";
        } else if (gamepad1.y) {
            robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);
            motor = "front left";
        } else if (gamepad1.x) {
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(power);
            robot.backRightMotor.setPower(0);
            motor = "back left";
        }
        telemetry.addData("Motor: ", motor);
        telemetry.addData("Power:", power);
        telemetry.addData("front left: ", robot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("front right: ", robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("back left: ", robot.backLeftMotor.getCurrentPosition());
        telemetry.addData("back right: ", robot.backRightMotor.getCurrentPosition());

    }
}
