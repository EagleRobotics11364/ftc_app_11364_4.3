package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;

import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeBuffer;
import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip;
@TeleOp(name="League Meet 4 Teleop", group="Meet4")
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
            directions[i] *= slow ?0.50 : 1; // if slow is true, multiply by 0.25
        }

        robot.holonomic.run((reverse?-1:1)*directions[0], (reverse?-1:1)*directions[1], directions[2]);
        telemetry.addData("Drive slow", slow);

        if (gamepad1.a) reverse = false;
        else if (gamepad1.b) reverse = true;

        /*
        Tape Spool Section
         */
        if(gamepad2.left_bumper | gamepad2.right_bumper) {
            robot.dualTapeSpools.move(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
        } else if(gamepad2.dpad_up) {
            robot.dualTapeSpools.move(1);
        } else if(gamepad2.dpad_down) {
            robot.dualTapeSpools.move(-1);
        } else robot.dualTapeSpools.stop();

        /*
        Emergency Servo Release
         */
        if (gamepad2.a) {
            robot.teamMarkerServo.setPosition(0.15);
        }
        if (gamepad2.x) {
            robot.craterArm.setPosition(0.47);
        } else if (gamepad2.y) {
            robot.craterArm.setPosition(0);
        }

        /*
        Color Sensor Telemetry
         */
        telemetry.addData("Left red", robot.leftColorSensor.red());
        telemetry.addData("Right red", robot.rightColorSensor.red());
        telemetry.update();
  
    }



}
