package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;
import org.firstinspires.ftc.teamcode.library.sampling.VuforiaTFODSampler;

import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeBuffer;
import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip;

@TeleOp(name="League Meet 4 Teleop +TFOD", group="Meet4")
public class LeagueMeetTeleopTensorFlow extends OpMode {
    public BaseRobot robot;
    public VuforiaTFODSampler sampler;

    boolean slow = false;
    boolean reverse = false;

    public void init() {
        robot = new BaseRobot(hardwareMap);
        try {
            sampler = new VuforiaTFODSampler(hardwareMap, telemetry);
        } catch (VuforiaTFODSampler.VuforiaTFODException e) {

        }
        sampler.activate();
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

        /*
        Color Sensor Telemetry
         */

        Position goldPosition = Position.NULL;
        if (gamepad2.x) {
            telemetry.addData("pos", sampler.recognize(Position.LEFT));
            telemetry.update();
        } else if (gamepad2.y) {
            telemetry.addData("pos", sampler.recognize(Position.CENTER));
            telemetry.update();
        } else if (gamepad2.b) {
            telemetry.addData("pos", sampler.recognize(Position.RIGHT));
            telemetry.update();
        } else if (gamepad2.a) {
            telemetry.addData("pos", sampler.recognize(Position.NULL));
            telemetry.update();
        }

    }

    @Override
    public void stop() {
        super.stop();
        sampler.shutdown();
    }
}
