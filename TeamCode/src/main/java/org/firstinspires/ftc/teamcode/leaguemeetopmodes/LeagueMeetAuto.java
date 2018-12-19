package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.ColorOperations;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.IterableTelemetryMenu;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItem;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemBoolean;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemEnum;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemInteger;
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;
import org.firstinspires.ftc.teamcode.library.sampling.FieldSample;
import org.firstinspires.ftc.teamcode.library.sampling.VuforiaTFODSampler;


@Autonomous(name = "BETA: Encoder Autonomous", group = "LT")
public class LeagueMeetAuto extends LinearOpMode {

    // robot, core systems variables
    private BaseRobot robot;
    private VuforiaTFODSampler vuforiaTFODSampler;

    // action variables
    private Position startingPosition = Position.NULL;
    private boolean useVuforiaTFOD = false;
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

        runPreMatchTelemetryMenu();

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
        if (land) {
            robot.dualTapeSpools.move(1);
            sleep(7000);
            robot.dualTapeSpools.stop();
            strafeUsingEncoder(-2.5,0,0.3);
        }

        // delay
        sleep(secondsDelay*1000);

        // sample
        if (useVuforiaTFOD) doVuforiaSampling();
        else                doColorSensorSampling();

        if (startingPosition == Position.RIGHT) {
            if (dropTeamMarker) {
                if (goldSamplePosition == Position.LEFT) {
                    strafeUsingEncoder(2,32,0.8);
                    strafeUsingEncoder(0,-5,0.5);
                    turnUsingEncoder(90,0.6);
                    strafeUsingEncoder(-10,0,0.6);
                } else if (goldSamplePosition == Position.CENTER) {
                    strafeUsingEncoder(0,30,0.6);
                    turnUsingEncoder(45,0.6);
                } else if (goldSamplePosition == Position.RIGHT) {
                    strafeUsingEncoder(0,24,0.6);
                    robot.teamMarkerServo.setPosition(0.10);
                    if(parkInCrater) {
//                        drive(0.8,-1,0,2000);
//                        drive(0.5,0,0,2000);
//                        drive(0,-0.5, 0, 1000);
                    }
//                    drive(-0.8, 0, 0, 800);
                }
                sleep(1000);
                robot.teamMarkerServo.setPosition(0.10);
                if (parkInCrater) {
                    sleep(1000);
//                    drive(0,0,0.75, 350);
//                    drive(0,-1, 0, 1750);
////                    robot.craterArm.setPosition(0);
//                    sleep(1000);
                    if (goldSamplePosition == Position.LEFT) {
                        strafeUsingEncoder(10,-10,0.8);
                        turnUsingEncoder(-45,0.7);
                        strafeUsingEncoder(0,-62,1);

                    }
                }
            }
        } else if (startingPosition == Position.LEFT) {
            if (parkInCrater) {
                drive(0,0.5,0,800);
                robot.craterArm.setPosition(0);
                sleep(1000);
            }
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

    private void doColorSensorSampling() {
        double obtainedHueLeft;
        double obtainedHueRight;
        double measuredHue;

        strafeUsingEncoder(-12, 19,0.7);
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
            strafeUsingEncoder(15,0,0.4);
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
                strafeUsingEncoder(15,0,0.4);
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
        MenuItemBoolean i_useVuforia = new MenuItemBoolean("useVuforia", "Use Vuforia/TFOD", true);
        MenuItemInteger i_startDelay = new MenuItemInteger("startDelay", "Delay Before Sampling", 0, 0, 15);
        MenuItemBoolean i_parkInCrater = new MenuItemBoolean("craterPark", "Park in Crater", true);
        MenuItemBoolean i_dropTeamMarker = new MenuItemBoolean("marker", "Drop Team Marker", true);
        MenuItemBoolean i_land = new MenuItemBoolean("land", "Deploy from Lander", true);
        menu.add(i_startingPosition, i_useVuforia, i_startDelay, i_parkInCrater, i_dropTeamMarker, i_land);

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
        useVuforiaTFOD = i_useVuforia.getValue();
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
