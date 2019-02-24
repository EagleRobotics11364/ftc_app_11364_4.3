package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;

import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeBuffer;
import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip;
@TeleOp(name="State TeleOp", group="LT")
public class StateTeleop extends OpMode {
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
            directions[i] *= (gamepad1.left_bumper & gamepad1.right_bumper) ? 0.50 : 1; // if slow is true, multiply by 0.25
        }

        robot.holonomic.runWithoutEncoder((reverse?-1:1)*directions[0], (reverse?-1:1)*directions[1], directions[2]);
//        telemetry.addData("Drive slow", slow);

        if (gamepad1.a) reverse = false;
        else if (gamepad1.b) reverse = true;

        /*
        Tape Spool Section
         */

        if (gamepad2.right_bumper) { // do hanging
            if (gamepad2.dpad_up) robot.hangingScrew.up();
            else if (gamepad2.dpad_down) robot.hangingScrew.halfDown();
            else robot.hangingScrew.manualControl(-gamepad2.right_stick_y);
        } else {
            robot.hangingScrew.stop();
            robot.intakeCubeServo.setPosition(gamepad2.right_trigger * 0.35);
            robot.intakeBallServo.setPosition(gamepad2.left_trigger * 0.35);
            robot.intakeArmMotor.setPower(gamepad2.left_stick_y);
            if (gamepad2.left_bumper) {
                robot.intakeArmExtender.setPower(gamepad2.right_stick_y);
                robot.mineralCollector.setPower(0);
            }
            else {
                robot.intakeArmExtender.setPower(0);
                robot.mineralCollector.setPower(gamepad2.right_stick_y);
            }
            if (gamepad2.a)
                robot.teamMarkerServo.setPosition(0);
            else if (gamepad2.b)
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
