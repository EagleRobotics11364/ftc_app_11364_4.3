package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;

import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeBuffer;
import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip;
@TeleOp(name="LT TeleOp", group="LT")
public class LeagueMeetTeleop extends OpMode {
    public BaseRobot robot;

    boolean slow = false;
    boolean reverse = false;

    public void init() {
        robot = new BaseRobot(hardwareMap);
    }

    public void loop() {
        /*
        Driving Section
         */
       if(gamepad1.dpad_up) slow = false;
       else if (gamepad1.dpad_down) slow = true;


        double directions[] = {gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x};

        for (int i = 0; i < directions.length; i++) {
            directions[i] = rangeClip(directions[i], -1, 1);
            directions[i] = rangeBuffer(directions[i], -0.1, 0.1, 0);
            directions[i] *= (gamepad1.left_bumper & gamepad1.right_bumper) ? (slow ? 0.50 : 1) : (slow ? 0.25 : 0.50); // if slow is true, multiply by 0.25
        }

        robot.holonomic.runWithoutEncoder((reverse?-1:1)*directions[0], (reverse?-1:1)*directions[1], directions[2]);
//        telemetry.addData("Drive slow", slow);

        if (gamepad1.a) reverse = false;
        else if (gamepad1.b) reverse = true;

        /*
        Tape Spool Section
         */
        if (!gamepad2.left_bumper) robot.intakeArmMotor.setPower(0);
        if(gamepad2.right_bumper) {
            robot.hangingScrew.setPower(-gamepad2.left_stick_y);
            if (gamepad2.left_bumper) robot.intakeArmMotor.setPower(-0.34);
        } else if (gamepad2.right_trigger>0.03) {
            robot.intakeArmHoldServo.setPosition(0.30);
            robot.intakeArmMotor.setPower(gamepad2.right_trigger);
        } else if(gamepad2.left_bumper) {
            double input = gamepad2.left_stick_y;
            robot.intakeArmHoldServo.setPosition(0.30);
            telemetry.addData("original input", input);
//            if (input > 0) input /= 5;
//            if (input != 1)
                if (input > -0.04) { // going down
                    input = 0.19 * input - 0.20;

                } else { // going up
                    input = 1 * input - 0.20;
                }
//            input = input * 0.5 - 0.20;
            telemetry.addData("final input", input);
            robot.intakeArmMotor.setPower(input);
        }
        else robot.hangingScrew.setPower(0);

        if (gamepad2.x) {
            robot.intakeBallServo.setPosition(0.95);
        } else if (gamepad2.y) {
            robot.intakeBallServo.setPosition(0.88);
        }

        /*
        Emergency Servo Release
         */
        if (gamepad2.a) {
            robot.teamMarkerServo.setPosition(0);
        } if (gamepad2.b) {
            robot.teamMarkerServo.setPosition(0.20);
        }


        /*
        Color Sensor Telemetry
         */
        telemetry.addData("Left red", robot.leftColorSensor.red());
        telemetry.addData("Right red", robot.rightColorSensor.red());
        telemetry.update();

    }



}
