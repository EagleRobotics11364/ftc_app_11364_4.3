package org.firstinspires.ftc.teamcode.leaguemeetopmodes.meet3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.ColorOperations;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.sampling.FieldSample;
import org.firstinspires.ftc.teamcode.library.sampling.VuforiaTFODSampler;
import org.firstinspires.ftc.teamcode.library.systems.LeagueMeet3Robot;


@Autonomous(name = "League Meet 3 Auto", group = "Meet3")
public class LeagueMeet3Auto extends LinearOpMode {

    // robot, core systems variables
    private LeagueMeet3Robot robot;
    private VuforiaTFODSampler vuforiaTFODSampler;

    // action variables
    private Position startingPosition = Position.NULL;
    private boolean useVuforiaTFOD = true;
    private int secondsDelay = 0;
    private boolean secondsDPadButtonStillPressed = false;

    // learned variables
    private Position goldSamplePosition = Position.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        // init robot
        robot = new LeagueMeet3Robot(hardwareMap);
        robot.teamMarkerServo.setPosition(0);
        while (!isStarted()) {
            // get presets for program
            if (gamepad1.dpad_left) startingPosition = Position.LEFT;
            if (gamepad1.dpad_right) startingPosition = Position.RIGHT;
            if (gamepad1.a) useVuforiaTFOD = true;
            if (gamepad1.b) useVuforiaTFOD = false;

            if (gamepad1.dpad_up && secondsDelay < 15) {
                secondsDelay++;
                while(gamepad1.dpad_up);
            } else if (gamepad1.dpad_down  && secondsDelay >  0) {
                secondsDelay--;
                while(gamepad1.dpad_down);
            } else {
                secondsDPadButtonStillPressed = false;
            }

            telemetry.addData("Starting Position", startingPosition);
            telemetry.addData("Use Vuforia/TFOD", useVuforiaTFOD);
            telemetry.addData("Sleep", secondsDelay);
            telemetry.update();
        }

        if (useVuforiaTFOD) {
            try {
                vuforiaTFODSampler = new VuforiaTFODSampler(hardwareMap, telemetry);
                vuforiaTFODSampler.activate();
                telemetry.addData("Vuforia", "Ready");
            } catch (VuforiaTFODSampler.UnsupportedHardwareException e) {
                useVuforiaTFOD = false;
                telemetry.addData("Vuforia", "Invalid Hardware!!!");
            }
            telemetry.update();
        }

        // land
/*        robot.dualTapeSpools.move(1);
        sleep(14000);*/

        // delay
        sleep(secondsDelay*1000);

        // sample
        if (useVuforiaTFOD) doVuforiaSampling();
        else                doColorSensorSampling();

        // team marker
        if (startingPosition == Position.RIGHT) {
            if (goldSamplePosition == Position.LEFT) drive(0,0,0,0); //set actual values
            else if (goldSamplePosition == Position.CENTER) drive(0,0,0,0); //set actual values
            else if (goldSamplePosition == Position.RIGHT) drive(0,0,0,0); //set actual values
            robot.teamMarkerServo.setPosition(0.10);
        }

        // park in crater
        if (startingPosition == Position.LEFT) {
            drive(0,0.3,0,500);
        }
    }

    private void doVuforiaSampling() {
        try {
            goldSamplePosition = vuforiaTFODSampler.recognize();
            switch(goldSamplePosition) {
                case LEFT:
                    drive(0.2, -0.6, 0, 1);
                    break;
                case CENTER:
                    drive(0,-0.5, 0, 1);
                    break;
                case RIGHT:
                    drive(-0.2, -0.6, 0, 1);
                    break;
                case NULL:
                    doColorSensorSampling();
            }
            vuforiaTFODSampler.shutdown();
        } catch (NullPointerException e) {
            doColorSensorSampling();
        }

    }

    private void doColorSensorSampling() {
        double obtainedHueLeft;
        double obtainedHueRight;
        double measuredHue;

        drive(-0.2, -0.6, 0, 1000);
        robot.holonomic.run(-0.2,-0.3,0);
        while(colorSensorsAreNotNaN());
        robot.holonomic.stop();
        sleep(200);

        obtainedHueLeft = ColorOperations.calculateHue(robot.leftColorSensor);
        obtainedHueRight = ColorOperations.calculateHue(robot.rightColorSensor);
        measuredHue = obtainedHueLeft < obtainedHueRight ? obtainedHueLeft : obtainedHueRight; //get lowest read hue value of both sensors

        if (measuredHue < FieldSample.HUE_MAXIMUM_GOLD) {
            goldSamplePosition = Position.LEFT;
        } else {
            robot.holonomic.run(-0.6,0,0);
            sleep(300);
            robot.holonomic.run(-0.1,0,0);
            while(colorSensorsAreNotNaN());
            robot.holonomic.stop();

            obtainedHueLeft = ColorOperations.calculateHue(robot.leftColorSensor);
            obtainedHueRight = ColorOperations.calculateHue(robot.rightColorSensor);
            measuredHue = obtainedHueLeft < obtainedHueRight ? obtainedHueLeft : obtainedHueRight; //get lowest read hue value of both sensors

            if (measuredHue < FieldSample.HUE_MAXIMUM_GOLD) {
                goldSamplePosition = Position.CENTER;
            } else {
                goldSamplePosition = Position.RIGHT;
                drive(-0.6,0,0,500);
            }

        }
    }

    private boolean colorSensorsAreNotNaN() {
        return (Double.isNaN(robot.leftDistanceSensor.getDistance(DistanceUnit.CM)) || Double.isNaN(robot.rightDistanceSensor.getDistance(DistanceUnit.CM)));
    }

    private void drive(double x, double y, double z, long duration) {
        robot.holonomic.run(x, y, z);
        sleep(duration);
        robot.holonomic.stop();
    }
}
