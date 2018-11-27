package org.firstinspires.ftc.teamcode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.sampling.VuforiaTFODSampler;

@Autonomous(name="Vuforia Sampling Test", group="Test")
public class VuforiaSamplerTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Position viewingDirection = Position.NULL;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) viewingDirection = Position.LEFT;
            else if (gamepad1.y) viewingDirection = Position.CENTER;
            else if (gamepad1.b) viewingDirection = Position.RIGHT;
            else if (gamepad1.a) viewingDirection = Position.NULL;
            telemetry.addData("view", viewingDirection);
            telemetry.update();
        }
        if (!isStopRequested() && opModeIsActive()) {
            try {
                Position position = Position.NULL;
                VuforiaTFODSampler sampler = new VuforiaTFODSampler(hardwareMap, telemetry);
                sampler.activate();
                sleep(2000);
                position = sampler.recognize(viewingDirection);
                telemetry.addData("Vuforia", position);
                telemetry.update();
                sampler.shutdown();
                sleep(3000);
            } catch (VuforiaTFODSampler.UnsupportedHardwareException e) {
                telemetry.addData("Vuforia", "Unsupported!!!");
                telemetry.update();
            }
        }

    }
}
