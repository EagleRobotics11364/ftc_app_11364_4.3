package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.ColorOperations;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.IterableTelemetryMenu;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemBoolean;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemEnum;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemInteger;
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;
import org.firstinspires.ftc.teamcode.library.sampling.FieldSample;
import org.firstinspires.ftc.teamcode.library.sampling.SamplingMethod;
import org.firstinspires.ftc.teamcode.library.sampling.TFODFilteringSampler;
import org.firstinspires.ftc.teamcode.library.sampling.VuforiaTFODSampler;


@Autonomous(name = "BETA: Encoder Autonomous", group = "LT")
public class LeagueMeetAuto extends LinearOpMode {

    // robot, core systems variables
    private BaseRobot robot;
    private VuforiaTFODSampler vuforiaTFODSampler;

    // action variables
    private Position startingPosition = Position.NULL;
    private SamplingMethod samplingMethod = SamplingMethod.COLORSENSORS;
    private boolean parkInCrater = false;
    private boolean dropTeamMarker = false;
    private boolean land = true;
    private int secondsDelay = 0;
    private IterableTelemetryMenu menu;

    // learned variables
    private Position goldSamplePosition = Position.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        // init robot
        robot = new BaseRobot(hardwareMap);
        robot.teamMarkerServo.setPosition(0);
        robot.craterArm.setPosition(0.47);
        robot.intakeArmHoldServo.setPosition(0);
        runPreMatchTelemetryMenu();

        if (samplingMethod != SamplingMethod.COLORSENSORS) {
            try {
                vuforiaTFODSampler = new VuforiaTFODSampler(hardwareMap, telemetry);
                vuforiaTFODSampler.activate();
                telemetry.addData("Vuforia", "Ready");
                TFODFilteringSampler tensorFlowSampler = new TFODFilteringSampler();
                tensorFlowSampler.recognizeGoldUsingTwoMinerals(Position.NULL);
            } catch (VuforiaTFODSampler.UnsupportedHardwareException e) {
                samplingMethod = SamplingMethod.COLORSENSORS;
                telemetry.addData("Vuforia", "Invalid Hardware!!!");
            }
            telemetry.update();
        }

        // land
        if (land) {
//            robot.dualTapeSpools.move(1);
            sleep(7000);
//            robot.dualTapeSpools.stop();
            strafeUsingEncoder(-4.25,0,0.6);
        }

        // delay
        sleep(secondsDelay*1000);
        telemetry.setAutoClear(false);
        // sample
        if (samplingMethod == SamplingMethod.TENSORFLOW) doVuforiaSampling();
        else if (samplingMethod == SamplingMethod.COLORSENSORS) doColorSensorSampling();
        else doCombinedSampling();

        telemetry.addData("Gold Sample Position", goldSamplePosition);
        telemetry.update();
        telemetry.setAutoClear(true);

        switch (startingPosition) {
            case LEFT:                      // START LEFT POSITION
                strafeUsingEncoder(0,goldSamplePosition==Position.RIGHT?12:7,0.6);
                if (dropTeamMarker) {       // start left position -> DROP TEAM MARKER
                    strafeUsingEncoder(0,-9,0.6);
                    int baselineStrafe = -35;
                    switch (goldSamplePosition) {
                        case LEFT:          // start left position -> drop team marker -> LEFT SAMPLE POSITION
                            // Our baseline position. Only strafe x = -20
                            strafeUsingEncoder(baselineStrafe,0,1);
                            break;
                        case CENTER:        // start left position -> drop team marker -> CENTER SAMPLE POSITION
                            // Strafe baseline along with -14.5 in.
                            strafeUsingEncoder(baselineStrafe + -14.5,0,1);
                            break;
                        case RIGHT:         // start left position -> drop team marker -> RIGHT SAMPLE POSITION
                            // Strafe baseline and center, along with -14.5 in.
                            strafeUsingEncoder(baselineStrafe + -14.5 + -14.5,0, 1);
                            break;
                    }
                    turnUsingEncoder(-45,0.8);
                    strafeUsingEncoder(0,20,0.6);
                    strafeUsingEncoder(-40,3.5,1);
                    robot.teamMarkerServo.setPosition(0.10);
                    turnUsingEncoder(-30,0.6);
                    sleep(200);
                    turnUsingEncoder(30,0.6);
                    sleep(200);
                    strafeUsingEncoder(40,4,1);
                    strafeUsingEncoder(0, -7,1);
                    turnUsingEncoder(90,1);
                    if (parkInCrater) {
                        strafeUsingEncoder(0,13,1);
                        robot.intakeArmHoldServo.setPosition(0.30);
                        robot.intakeArmMotor.setPower(-0.7);
                        sleep(1000);
                        robot.intakeArmMotor.setPower(-0.20);
                    }


                } else if (parkInCrater) { // start left position -> PARK IN CRATER
                    strafeUsingEncoder(0,10,0.65);
                }
                break;
            case RIGHT:                     // START RIGHT POSITION
                if (dropTeamMarker) {       // start right position -> DROP TEAM MARKER
                    switch (goldSamplePosition) {
                        case LEFT:          // start right position -> drop team marker -> LEFT SAMPLE POSITION
                            strafeUsingEncoder(-5.5,32,0.8);
                            strafeUsingEncoder(0,-5,0.5);
                            turnUsingEncoder(90,0.6);
                            strafeUsingEncoder(-10,0,0.6);
                            break;
                        case CENTER:        // start right position -> drop team marker -> CENTER SAMPLE POSITION
                            strafeUsingEncoder(0,30,0.6);
                            turnUsingEncoder(45,0.6);
                            robot.teamMarkerServo.setPosition(0.10);
                            strafeUsingEncoder(-14,-14,0.8);
                            break;
                        case RIGHT:         // start right position -> drop team marker -> RIGHT SAMPLE POSITION
                            strafeUsingEncoder(0,30,0.8);
                            turnUsingEncoder(45,0.6);
                            strafeUsingEncoder(-22,0,0.7);
                            robot.teamMarkerServo.setPosition(0.10);
                            sleep(500);
                            strafeUsingEncoder(-6,-10,0.7);
                            break;
                    }

                    sleep(1000);
                    robot.teamMarkerServo.setPosition(0.10);

//                    if (parkInCrater) {     // start right position -> drop team marker -> any sample position -> PARK IN CRATER
                        sleep(1000);

                        switch(goldSamplePosition) {
                            case LEFT:      // start right position -> drop team marker -> any sample position -> park in crater -> LEFT SAMPLE POSITION
                                strafeUsingEncoder(10,-10,0.8);
                                turnUsingEncoder(-45,0.7);
                            case CENTER:    // start right position -> drop team marker -> any sample position ->park in crater -> CENTER SAMPLE POSITION
                            case RIGHT:     // start right position -> drop team marker -> any sample position ->park in crater -> RIGHT SAMPLE POSITION
                                strafeUsingEncoder(0,-40,1);
                                break;
                        }
                        if (parkInCrater) {
                            turnUsingEncoder(150, 1);
                            sleep(500);
                            robot.intakeArmHoldServo.setPosition(0.30);
                            robot.intakeArmMotor.setPower(-0.7);
                            sleep(1000);
                            robot.intakeArmMotor.setPower(-0.20);
//                        } else if(gamepad2.left_bumper) {
//                            double input = gamepad2.left_stick_y;
//
//                            telemetry.addData("original input", input);
//                            if (input > -0.04) { // going down
//                                input = 0.19 * input - 0.20;
//
//                            } else { // going up
//                                input = 1 * input - 0.20;
//                            }
////            input = input * 0.5 - 0.20;
//                        }
////                    }
                        }
                }
                break;
        }
        sleep(1000);
        /*


         */

    }

    private void doVuforiaSampling() {
        try {
            goldSamplePosition = vuforiaTFODSampler.recognize();
            switch(goldSamplePosition) {
                case LEFT:
                    drive(0.2, 0.6, 0, 1);
                    break;
                case CENTER:
                    drive(0,0.5, 0, 1);
                    break;
                case RIGHT:
                    drive(-0.4, 0.6, 0, 1);
                    break;
                case NULL:
                    doColorSensorSampling();
            }
            vuforiaTFODSampler.shutdown();
        } catch (NullPointerException e) {
            doColorSensorSampling();
        }

    }

    private void doCombinedSampling() {
        double obtainedHueLeft;
        double obtainedHueRight;
        double measuredHue;
        VuforiaTFODSampler.SamplerResult tfResult = vuforiaTFODSampler.recognizeUsingGoldLocation_viewRightPosition();
        for (int i = 0; i < 4; i++) {
            if (tfResult.position != Position.NULL) break;
            sleep(200);
            tfResult = vuforiaTFODSampler.recognizeUsingGoldLocation_viewRightPosition();
        }

        strafeUsingEncoder(-10.75, 19,0.7);
        sleep(800);
        obtainedHueLeft = ColorOperations.calculateHue(robot.leftColorSensor);
        obtainedHueRight = ColorOperations.calculateHue(robot.rightColorSensor);
        measuredHue = obtainedHueLeft < obtainedHueRight ? obtainedHueLeft : obtainedHueRight; //get lowest read hue value of both sensors
        telemetry.addData("left hue", obtainedHueLeft);
        telemetry.addData("right hue", obtainedHueRight);
        telemetry.addData("measured hue", measuredHue);
        telemetry.update();
        if (measuredHue < FieldSample.HUE_MAXIMUM_GOLD && Math.abs(obtainedHueLeft-obtainedHueRight) < 20) {
            goldSamplePosition = Position.LEFT;
        } else if ((measuredHue < FieldSample.HUE_MAXIMUM_GOLD && Math.abs(obtainedHueLeft-obtainedHueRight) < 40) || measuredHue < FieldSample.HUE_GOLD_EXTENSION) {
            if (tfResult.position == Position.LEFT && tfResult.confidence > 0.5) goldSamplePosition = Position.LEFT;
        } else if (Math.abs(obtainedHueLeft-obtainedHueRight) < 60) {
            if (tfResult.position == Position.LEFT && tfResult.confidence > 0.4) goldSamplePosition = Position.LEFT;
        }
        if (goldSamplePosition == Position.NULL){
            strafeUsingEncoder(15,-1,0.4);
            sleep(800);
            obtainedHueLeft = ColorOperations.calculateHue(robot.leftColorSensor);
            obtainedHueRight = ColorOperations.calculateHue(robot.rightColorSensor);
            measuredHue = obtainedHueLeft < obtainedHueRight ? obtainedHueLeft : obtainedHueRight; //get lowest read hue value of both sensors
            telemetry.addData("left hue", obtainedHueLeft);
            telemetry.addData("right hue", obtainedHueRight);
            telemetry.addData("measured hue", measuredHue);
            telemetry.update();

            if (measuredHue < FieldSample.HUE_MAXIMUM_GOLD && Math.abs(obtainedHueLeft-obtainedHueRight) < 20) {
                goldSamplePosition = Position.CENTER;
            } else if ((measuredHue < FieldSample.HUE_MAXIMUM_GOLD && Math.abs(obtainedHueLeft-obtainedHueRight) < 40) || measuredHue < FieldSample.HUE_GOLD_EXTENSION) {
                if (tfResult.position == Position.CENTER && tfResult.confidence > 0.5) goldSamplePosition = Position.CENTER;
            } else if (Math.abs(obtainedHueLeft-obtainedHueRight) < 60) {
                if (tfResult.position == Position.CENTER && tfResult.confidence > 0.4) goldSamplePosition = Position.CENTER;
            }
            if (goldSamplePosition == Position.NULL){
                goldSamplePosition = Position.RIGHT;
                strafeUsingEncoder(18,-1,0.4);
            }

        }
    }

    private void doColorSensorSampling() {
        double obtainedHueLeft;
        double obtainedHueRight;
        double measuredHue;

        strafeUsingEncoder(-10.75, 19,0.85);
        sleep(800);

        obtainedHueLeft = ColorOperations.calculateHue(robot.leftColorSensor);
        obtainedHueRight = ColorOperations.calculateHue(robot.rightColorSensor);
        measuredHue = obtainedHueLeft < obtainedHueRight ? obtainedHueLeft : obtainedHueRight; //get lowest read hue value of both sensors
        telemetry.addData("left hue", obtainedHueLeft);
        telemetry.addData("right hue", obtainedHueRight);
        telemetry.addData("measured hue", measuredHue);
        telemetry.update();
        if (measuredHue < FieldSample.HUE_MAXIMUM_GOLD) {
            goldSamplePosition = Position.LEFT;
        } else {
            strafeUsingEncoder(15,-1,0.6);
            sleep(800);
            obtainedHueLeft = ColorOperations.calculateHue(robot.leftColorSensor);
            obtainedHueRight = ColorOperations.calculateHue(robot.rightColorSensor);
            measuredHue = obtainedHueLeft < obtainedHueRight ? obtainedHueLeft : obtainedHueRight; //get lowest read hue value of both sensors
            telemetry.addData("left hue", obtainedHueLeft);
            telemetry.addData("right hue", obtainedHueRight);
            telemetry.addData("measured hue", measuredHue);
            telemetry.update();
            if (measuredHue < FieldSample.HUE_MAXIMUM_GOLD) {
                goldSamplePosition = Position.CENTER;
            } else {
                goldSamplePosition = Position.RIGHT;
                strafeUsingEncoder(18,-1,0.6);
            }

        }
    }

    private boolean colorSensorsAreNaN() {
        return (Double.isNaN(robot.leftDistanceSensor.getDistance(DistanceUnit.CM)) || Double.isNaN(robot.rightDistanceSensor.getDistance(DistanceUnit.CM)));
    }

    private void drive(double x, double y, double z, long duration) {
        robot.holonomic.runWithoutEncoder(x, y, z);
        sleep(duration);
        robot.holonomic.stop();
    }

    private void runPreMatchTelemetryMenu() {
        menu = new IterableTelemetryMenu(telemetry);
        MenuItemEnum<Position> i_startingPosition = new MenuItemEnum<Position>("startingPosition", "Starting Position", Position.LEFT, Position.RIGHT);
        MenuItemEnum<SamplingMethod> i_samplingMethod = new MenuItemEnum<SamplingMethod>("samplingMethod", "Sampling Method", SamplingMethod.COLORSENSORS, SamplingMethod.COMBINED, SamplingMethod.TENSORFLOW);
        MenuItemInteger i_startDelay = new MenuItemInteger("startDelay", "Delay Before Sampling", 0, 0, 15);
        MenuItemBoolean i_parkInCrater = new MenuItemBoolean("craterPark", "Park in Crater", true);
        MenuItemBoolean i_dropTeamMarker = new MenuItemBoolean("marker", "Drop Team Marker", true);
        MenuItemBoolean i_land = new MenuItemBoolean("land", "Deploy from Lander", true);
        menu.add(i_startingPosition, i_samplingMethod, i_startDelay, i_parkInCrater, i_dropTeamMarker, i_land);

        while (!isStarted()&&!isStopRequested()) {
            if (gamepad1.dpad_up) {
                menu.previousItem();
                while (gamepad1.dpad_up) ;
            } else if (gamepad1.dpad_down) {
                menu.nextItem();
                while (gamepad1.dpad_down) ;
            } else if (gamepad1.dpad_left) {
                menu.iterateBackward();
                while (gamepad1.dpad_left) ;
            } else if (gamepad1.dpad_right) {
                menu.iterateForward();
                while (gamepad1.dpad_right) ;
            }
        }
        if (isStopRequested()) return;
        startingPosition = i_startingPosition.getValue();
        samplingMethod = i_samplingMethod.getValue();
        parkInCrater = i_parkInCrater.getValue();
        dropTeamMarker = i_dropTeamMarker.getValue();
        secondsDelay = i_startDelay.getValue();
        land = i_land.getValue();
    }

    private void strafeUsingEncoder(double x, double y, double power) {
        robot.holonomic.runUsingEncoder(x, y, power);
        while (opModeIsActive() & robot.holonomic.motorsAreBusy());
    }
    private void turnUsingEncoder(double degrees, double power) {
        robot.holonomic.turnUsingEncoder(degrees,power);
        while (opModeIsActive() & robot.holonomic.motorsAreBusy());
    }

}
