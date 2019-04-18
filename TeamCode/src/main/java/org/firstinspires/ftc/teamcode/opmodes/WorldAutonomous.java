package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.ColorOperations;
import org.firstinspires.ftc.teamcode.library.functions.MusicPlayer;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.IterableTelemetryMenu;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemBoolean;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemEnum;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemInteger;
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;
import org.firstinspires.ftc.teamcode.library.robot.systems.IMUDrivetrainControllerKotlin;
import org.firstinspires.ftc.teamcode.library.sampling.FieldSample;
import org.firstinspires.ftc.teamcode.library.sampling.SamplingMethod;
import org.firstinspires.ftc.teamcode.library.sampling.TensorFlowSampler;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "World Autonomous", group = "OpModes")
public class WorldAutonomous extends LinearOpMode {

    // robot, core systems variables
    private BaseRobot robot;
    private TensorFlowSampler tensorFlowSampler;
    private IMUDrivetrainControllerKotlin imuController;

    // action variables
    private Position startingPosition = Position.NULL;
    private SamplingMethod samplingMethod = SamplingMethod.COLORSENSORS;
    private boolean parkInCrater = false;
    private boolean dropTeamMarker = false;
    private boolean land = true;
    private boolean useIMU = false;
    private boolean useLEDs = true;
    private boolean useMusic = false;
    private int secondsDelay = 0;
    private IterableTelemetryMenu menu;
    private MusicPlayer musicPlayer;

    // learned variables
    private Position goldSamplePosition = Position.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        // init robot
        robot = new BaseRobot(hardwareMap);
        musicPlayer = new MusicPlayer(hardwareMap);
        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        robot.teamMarkerServo.setPosition(BaseRobot.TEAM_MARKER_RETRACTED);
//        robot.craterArm.setPosition(0.47);
        runPreMatchTelemetryMenu();

        if (useIMU) {
            imuController = new IMUDrivetrainControllerKotlin(robot.imuA, robot.imuB, robot.holonomic, telemetry);
            reportIMUData();
        }
        if (useMusic) musicPlayer.play();
        switch (samplingMethod) {
            case TFOD_TRIPLE: case TFOD_DUAL: case TFOD_SINGLE:
            try {
                telemetry.addData("Vuforia", "Ready");
                tensorFlowSampler = TensorFlowSampler.Factory.newSampler(hardwareMap);
                tensorFlowSampler.activate();
            } catch (TensorFlowSampler.UnsupportedHardwareException e) {
                samplingMethod = SamplingMethod.COLORSENSORS;
                telemetry.addData("Vuforia", "Invalid Hardware!!!");
            }
            telemetry.update();
            break;
        }

        if (samplingMethod == SamplingMethod.TFOD_TRIPLE) {
            for (int i = 0; i<5 & goldSamplePosition == Position.NULL; i++) {
                goldSamplePosition = tensorFlowSampler.recognizeGoldUsingThreeMinerals();
                sleep(100);
            }
            switch (goldSamplePosition) {
                case LEFT:
                    setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                case CENTER:
                    setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                case RIGHT:
                    setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                case NULL:
                    setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }
        }

        // land
        if (land) {
           robot.hangingScrew.up();
            sleep(750);
            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
            sleep(1750);
        }
        strafeUsingEncoder(-4.5,0.5,0.6);
        // delay
        if (secondsDelay != 0) {
            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW);
            sleep(secondsDelay*1000);
        }
        telemetry.setAutoClear(false);
        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
        // sample
        switch (samplingMethod) {
             case TFOD_TRIPLE:
                switch (goldSamplePosition) {
                    case LEFT:
                        telemetry.addLine("Gold sample is on left");
                        telemetry.update();
                        strafeUsingEncoder(-9, 22, 1 );
                        tensorFlowSampler.deactivate();
                        break;
                    case CENTER:
                        telemetry.addLine("Gold sample is on center");
                        telemetry.update();
                        strafeUsingEncoder(0, 22, 1 );
                        tensorFlowSampler.deactivate();
                        break;
                    case RIGHT:
                        telemetry.addLine("Gold sample is on right");
                        telemetry.update();
                        strafeUsingEncoder(15, 22, 1 );
                        tensorFlowSampler.deactivate();
                        break;
                    case NULL:
                        telemetry.addLine("Could not find sample while hanging");
                        telemetry.addLine("Sampling two samples...");
                        telemetry.update();
                        doTensorFlowDualSampling();
                        break;
                }
                break;
            case TFOD_DUAL:
                doTensorFlowDualSampling();
                tensorFlowSampler.deactivate();
                telemetry.addLine(tensorFlowSampler.getOutput());
                telemetry.update();
                break;
            case TFOD_SINGLE:
                doTensorFlowSingleSampling();
                tensorFlowSampler.deactivate();
                break;
            case ALWAYS_CENTER:
                strafeUsingEncoder(3,19,0.9);
                goldSamplePosition = Position.CENTER;
                break;
            default:
                doColorSensorSampling(true);
                break;
        }
//        else doCombinedSampling();

        telemetry.addData("Gold Sample Position", goldSamplePosition);
        telemetry.update();
        telemetry.setAutoClear(true);

        switch (startingPosition) {
            case LEFT:                      // START LEFT POSITION
                strafeUsingEncoder(0,9,1);
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
                if (dropTeamMarker) {       // start left position -> DROP TEAM MARKER
                    strafeUsingEncoder(0,-10,1);
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
                    drive(0,1,0,750);
                    sleep(250);
                    strafeUsingEncoder(0,-3.0, 1 );
                    switch (goldSamplePosition) {
                        case LEFT: case CENTER:
                            strafeUsingEncoder(-48,0,1);
                            break;
                        case RIGHT:
                            strafeUsingEncoder(-55,0,1);
                    }

                    strafeUsingEncoder(0,-3,0.8);
                    sleep(250);
                    robot.teamMarkerServo.setPosition(BaseRobot.TEAM_MARKER_RELEASED);
//                    turnUsingEncoder(-30,0.6);
//                    sleep(200);
//                    turnUsingEncoder(30,0.6);
                    sleep(300);
                    strafeUsingEncoder(50,4,1);
                    drive(0,1,0,500);
                    sleep(200);
                    strafeUsingEncoder(0, -4,1);
                    switch (goldSamplePosition) {
                        case LEFT: case CENTER:
                            strafeUsingEncoder(8,0,1 );
                            break;
                        case RIGHT:
                            break;
                    }
                    while (robot.intakeArmPivotController.getFullProportion() > 0.10 & opModeIsActive()) {
                        robot.intakeArmPivotController.controlFromGamepadInput(-0.5, false);
                    }


                } else if (parkInCrater) { // start left position -> PARK IN CRATER
                    strafeUsingEncoder(0,10,0.65);
                }
                break;
            case RIGHT:                     // START RIGHT POSITION
                if (dropTeamMarker) {       // start right position -> DROP TEAM MARKER
                    switch (goldSamplePosition) {
                        case LEFT:          // start right position -> drop team marker -> LEFT SAMPLE POSITION
                            strafeUsingEncoder(-5.5,20,1);
                            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
                            turnUsingEncoder(45,1);
                            strafeUsingEncoder(0,25,1);
                            sleep(1000);
                            robot.teamMarkerServo.setPosition(BaseRobot.TEAM_MARKER_RELEASED);
                            break;
                        case CENTER:        // start right position -> drop team marker -> CENTER SAMPLE POSITION
                            strafeUsingEncoder(-2,30,0.6);
                            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
                            turnUsingEncoder(120,0.6);
                            strafeUsingEncoder(0,-30,1);
                            drive(0, -1, 0, 1000);
                            strafeUsingEncoder(0,3,1);
                            robot.teamMarkerServo.setPosition(BaseRobot.TEAM_MARKER_RELEASED);
                            strafeUsingEncoder(30,0,0.8);
                            break;
                        case RIGHT:         // start right position -> drop team marker -> RIGHT SAMPLE POSITION
                            strafeUsingEncoder(0,20,0.8);
                            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
                            turnUsingEncoder(45,0.6);
                            strafeUsingEncoder(-22,0,0.7);
                            robot.teamMarkerServo.setPosition(BaseRobot.TEAM_MARKER_RELEASED);
                            sleep(500);
                            strafeUsingEncoder(-6,-10,0.7);
                            break;
                    }



//
                        sleep(1000);
                        switch(goldSamplePosition) {
                            case LEFT:      // start right position -> drop team marker -> any sample position -> park in crater -> LEFT SAMPLE POSITION
//                                strafeUsingEncoder(10,-10,0.8);
                                strafeUsingEncoder(0,-40,1);
                                break;
                            case CENTER:// start right position -> drop team marker -> any sample position -> park in crater -> CENTER SAMPLE POSITION
                                strafeUsingEncoder(30,0,1);
                                break;
                            case RIGHT:     // start right position -> drop team marker -> any sample position -> park in crater -> RIGHT SAMPLE POSITION
                                turnUsingEncoder(90,1 );
                                drive(0,-1,0, 1000);
                                sleep(200);
                                strafeUsingEncoder(0,3.5,1 );
                                strafeUsingEncoder(70,0,1);
                                break;
                        }
                    if (parkInCrater) {
                        while (robot.intakeArmPivotController.getFullProportion() > 0.10 & opModeIsActive()) {
                            robot.intakeArmPivotController.controlFromGamepadInput(-0.5, false);
                        }
                    }
                }
                break;
        }
        sleep(500);
       reportIMUData();

        /*


         */
        musicPlayer.stop();
    }

    private void doColorSensorSampling(boolean driveForward) {
        double obtainedHueLeft;
        double obtainedHueRight;
        double measuredHue;
        println("Beginning color sensor sampling...");
        if (driveForward) strafeUsingEncoder(-8, 18.5, 0.85);
        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
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
            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        } else {
            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            strafeUsingEncoder(15,-0.5,0.6);
            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
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
                strafeUsingEncoder(-6,0, 1);
            } else {
                goldSamplePosition = Position.RIGHT;
                strafeUsingEncoder(18,-1,0.6);
            }

        }
    }

    private void doTensorFlowSingleSampling() {
        FieldSample foundMineral = FieldSample.NULL;
        strafeUsingEncoder(0, 9, 1);
        println("strafed 9in fw");
        turnUsingEncoder(-30, 1);
        println("turning to left");
        foundMineral = tensorFlowSampler.recognizeOneMineral();
        println("at left: found: "+foundMineral);
        if (foundMineral  == FieldSample.GOLD) {
            goldSamplePosition = Position.LEFT;
            turnUsingEncoder(30,0.8);
            strafeUsingEncoder(-10, 9,1);
            println("going to gold @ left");
        } else {
            println("didn't find gold at left: going to center");
            turnUsingEncoder(30,0.6);
            robot.holonomic.stop();
            sleep(2000);
            foundMineral = tensorFlowSampler.recognizeOneMineral();
            println("at center: found: "+foundMineral);
            if (foundMineral == FieldSample.GOLD) {
                goldSamplePosition = Position.CENTER;

                strafeUsingEncoder(2, 10,1);
            } else {

                goldSamplePosition = Position.RIGHT;
                strafeUsingEncoder(22, 9, 1);
            }
        }
        telemetry.addLine("found gold at: " + goldSamplePosition);
    }

    private void doTensorFlowDualSampling() {
        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        sleep(300);
        for (int i = 0; i<5 && goldSamplePosition==Position.NULL; i++) {
            goldSamplePosition = tensorFlowSampler.recognizeGoldUsingTwoMinerals(Position.LEFT);
            sleep(100);
        }
        telemetry.addData("position", goldSamplePosition);
        telemetry.update();
        final int FWDISTANCE = 22;
        println("TFOD Dual-Sample Pass 1: " + goldSamplePosition.toString());
        switch (goldSamplePosition) {
            case LEFT:
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                println("Driving left after Pass 1 reading");
                strafeUsingEncoder(-9,FWDISTANCE, 1);
                break;
            case CENTER:
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                println("Driving center after Pass 1 reading");
                strafeUsingEncoder(0, FWDISTANCE,1);
                break;
            case RIGHT:
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                println("Driving right after Pass 1 reading");
                strafeUsingEncoder(15, FWDISTANCE, 0.7);
                break;
            case NULL:
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                println("Driving forward for Pass 2...");
                strafeUsingEncoder(-1.5, 8, 1);
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                sleep(300);
                for (int i = 0; i<5 && goldSamplePosition==Position.NULL; i++) {
                    goldSamplePosition = tensorFlowSampler.recognizeGoldUsingTwoMinerals(Position.LEFT);
                    sleep(100);
                }
                println("TFOD Dual-Sample Pass 2: " + goldSamplePosition.toString());
                switch (goldSamplePosition) {
                    case LEFT:
                        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                        println("Driving left after Pass 2 reading");
                        strafeUsingEncoder(-4,FWDISTANCE-8, 1);
                        break;
                    case CENTER:
                        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                        println("Driving center after Pass 2 reading");
                        strafeUsingEncoder(4, FWDISTANCE-8,1);
                        break;
                    case RIGHT:
                        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                        println("Driving right after Pass 2 reading");
                        strafeUsingEncoder(29, FWDISTANCE-8, 1);
                        break;
                    case NULL:
                        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                        strafeUsingEncoder(-4.5, 10,0.85);
                        doColorSensorSampling(false);
                }
        }
        tensorFlowSampler.deactivate();
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
        MenuItemEnum<SamplingMethod> i_samplingMethod = new MenuItemEnum<SamplingMethod>("samplingMethod", "Sampling Method", SamplingMethod.TFOD_TRIPLE, SamplingMethod.TFOD_DUAL, SamplingMethod.TFOD_SINGLE, SamplingMethod.COLORSENSORS, SamplingMethod.ALWAYS_CENTER);
        MenuItemInteger i_startDelay = new MenuItemInteger("startDelay", "Delay Before Sampling", 0, 0, 15);
        MenuItemBoolean i_parkInCrater = new MenuItemBoolean("craterPark", "Park in Crater", true);
        MenuItemBoolean i_dropTeamMarker = new MenuItemBoolean("marker", "Drop Team Marker", true);
        MenuItemBoolean i_land = new MenuItemBoolean("land", "Deploy from Lander", true);
        MenuItemBoolean i_useIMU = new MenuItemBoolean("useIMU","Enable IMU",false);
        MenuItemBoolean i_useLEDs = new MenuItemBoolean("useLEDs","Enable LEDs",true);
        MenuItemBoolean i_useMusic = new MenuItemBoolean("useMusic","Enable Music",false);
        menu.add(i_startingPosition, i_samplingMethod, i_startDelay, i_parkInCrater, i_dropTeamMarker, i_land, i_useIMU, i_useLEDs, i_useMusic);

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
        useIMU = i_useIMU.getValue();
        useLEDs = i_useLEDs.getValue();
        useMusic = i_useMusic.getValue();
    }

    private void strafeUsingEncoder(double x, double y, double power) {
        robot.holonomic.runUsingEncoder(x, y, power);
        while (opModeIsActive() & robot.holonomic.motorsAreBusy());
        reportIMUData();
    }
    private void turnUsingEncoder(double degrees, double power) {
        robot.holonomic.turnUsingEncoder(degrees,power);
        while (opModeIsActive() & robot.holonomic.motorsAreBusy());
        reportIMUData();
    }

    private void println(String str) {
        telemetry.setAutoClear(false);
        telemetry.addLine(str);
        telemetry.update();
    }

    private void reportIMUData() {
        if (useIMU) {
            sleep(200);
            telemetry.addData("headingA",imuController.getHeadingA());
            telemetry.addData("headingB", imuController.getHeadingB());
            telemetry.update();
            sleep(200);
        }

    }
    
    private void setLEDPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (useLEDs) robot.blinkin.setPattern(pattern);
    }
}
