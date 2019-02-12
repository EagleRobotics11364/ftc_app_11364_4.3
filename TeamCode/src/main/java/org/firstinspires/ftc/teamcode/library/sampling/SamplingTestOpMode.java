package org.firstinspires.ftc.teamcode.library.sampling;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.library.functions.Position;

public class SamplingTestOpMode extends OpMode {
    TensorFlowSampler tfod;
    int i = 0;
    @Override
    public void init() {
        try{
            tfod = TensorFlowSampler.Factory.newSampler(hardwareMap);
        } catch (TensorFlowSampler.UnsupportedHardwareException e) {
            stop();
        }
        tfod.activate();
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            while (gamepad1.x);
            i++;
            telemetry.addData("position"+i, tfod.recognizeGoldUsingTwoMinerals(Position.LEFT));
            telemetry.update();
        }
        else if (gamepad1.y) {
            while (gamepad1.y);
            i++;
            telemetry.addData("mineral"+i, tfod.recognizeOneMineral());
            telemetry.update();
        } else if (gamepad1.b) {
            while (gamepad1.b);
            i++;
            telemetry.addData("mineral"+i, tfod.recognizeGoldUsingTwoMinerals(Position.RIGHT));
            telemetry.update();
        } else if (gamepad1.a) {
            while (gamepad1.a);
            i++;
            telemetry.addData("position"+ i, tfod.recognizeGoldUsingThreeMinerals());
            telemetry.update();
        }
    }
    @Override
    public void stop() {
        super.stop();
        tfod.deactivate();
    }
}
