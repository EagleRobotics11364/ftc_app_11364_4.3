package org.firstinspires.ftc.teamcode.leaguemeetopmodes.meet3;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.systems.LeagueMeet3Robot;

import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeBuffer;
import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip;
@TeleOp(name="League Meet 3 Teleop", group="Meet3")
public class LeagueMeet3Teleop extends OpMode {
    public LeagueMeet3Robot robot;

    boolean slow = false;

    public void init() {
        robot = new LeagueMeet3Robot(hardwareMap);
    }

    public void loop() {
        /*
        Driving Section
         */
        if(gamepad1.dpad_up) slow = false;
        else if (gamepad1.dpad_down) slow = true;


        double directions[] = {gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x};

        for (int i = 0; i < directions.length; i++) {
            directions[i] = rangeClip(directions[i], -1, 1);
            directions[i] = rangeBuffer(directions[i], -0.1, 0.1, 0);
            directions[i] *= slow ?0.50 : 1; // if slow is true, multiply by 0.25
        }

        robot.holonomic.run(directions[0], directions[1], directions[2]);
        telemetry.addData("Drive slow", slow);

        /*
        Tape Spool Section
         */
        if(gamepad2.left_bumper | gamepad2.right_bumper) {
            robot.dualTapeSpools.move(gamepad2.left_stick_y, gamepad2.right_stick_y);
        } else if(gamepad2.dpad_up) {
            robot.dualTapeSpools.move(1);
        } else if(gamepad2.dpad_down) {
            robot.dualTapeSpools.move(-1);
        } else robot.dualTapeSpools.stop();

        /*
        Color Sensor Telemetry
         */
        telemetry.addData("Left red", robot.leftColorSensor.red());
        telemetry.addData("Right red", robot.leftColorSensor.red());
        telemetry.update();

    }



}
