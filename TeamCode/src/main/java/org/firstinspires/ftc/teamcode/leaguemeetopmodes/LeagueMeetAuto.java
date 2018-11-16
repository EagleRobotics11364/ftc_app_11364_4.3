package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.ColorOperations;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.IterableTelemetryMenu;
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;
import org.firstinspires.ftc.teamcode.library.sampling.FieldSample;
import org.firstinspires.ftc.teamcode.library.sampling.VuforiaTFODSampler;


@Autonomous(name = "League Meet 4 Autonomous", group = "Meet4")
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


    // learned variables
    private Position goldSamplePosition = Position.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        // init robot
        robot = new BaseRobot(hardwareMap);
        robot.teamMarkerServo.setPosition(0);
        robot.craterArm.setPosition(0.47);
        IterableMenu iterableMenu = new IterableMenu(telemetry);
        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                iterableMenu.previousItem();
                while (gamepad1.dpad_up) ;
            } else if (gamepad1.dpad_down) {
                iterableMenu.nextItem();
                while (gamepad1.dpad_down) ;
            } else if (gamepad1.dpad_left) {
                iterableMenu.iterateValueBackwards();
                while (gamepad1.dpad_left) ;
            } else if (gamepad1.dpad_right) {
                iterableMenu.iterateValueForward();
                while (gamepad1.dpad_right) ;
            }
        }

        startingPosition = iterableMenu.position.getValue();
        useVuforiaTFOD = iterableMenu.useVuforia.getValue();
        secondsDelay = iterableMenu.startDelay.getValue();
        parkInCrater = iterableMenu.parkInCrater.getValue();
        dropTeamMarker = iterableMenu.dropTeamMarker.getValue();
        land = iterableMenu.land.getValue();

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
            sleep(11000);
            robot.dualTapeSpools.stop();
            drive(-0.2,0,0,400);
        }

        // delay
        sleep(secondsDelay*1000);

        // sample
        if (useVuforiaTFOD) doVuforiaSampling();
        else                doColorSensorSampling();

        if (startingPosition == Position.RIGHT) {
            if (dropTeamMarker) {
                if (goldSamplePosition == Position.LEFT) {
                    drive(0.3,1,0,1500);
                } else if (goldSamplePosition == Position.CENTER) {
                    drive(0,1,0,1200);
                } else if (goldSamplePosition == Position.RIGHT) {
                    drive(-0.20,1,0,1200);
                    drive(-0.8, 0, 0, 800);
                }
                robot.teamMarkerServo.setPosition(0.15);
                if (parkInCrater) {
                    drive(0,0,0.75, 350);
                    drive(0,-1, 0, 1750);
                    robot.craterArm.setPosition(0);
                    sleep(1000);

                }
            }
        } else if (startingPosition == Position.LEFT) {
            if (parkInCrater) {
                drive(0,0.5,0,800);
                robot.craterArm.setPosition(0);
                sleep(1000);
            }
        }

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

        drive(-0.576, 0.96, 0, 500);
        robot.holonomic.run(-0.12*0.75,0.27*0.75,0);
        while(colorSensorsAreNaN());
        robot.holonomic.stop();
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
            robot.holonomic.run(0.6,-0.03,0);
            sleep(500);
            robot.holonomic.run(0.3,-0.03,0);
            while(colorSensorsAreNaN());
            robot.holonomic.stop();

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
                drive(1,-0.07,0,500);
            }

        }
    }

    private boolean colorSensorsAreNaN() {
        return (Double.isNaN(robot.leftDistanceSensor.getDistance(DistanceUnit.CM)) || Double.isNaN(robot.rightDistanceSensor.getDistance(DistanceUnit.CM)));
    }

    private void drive(double x, double y, double z, long duration) {
        robot.holonomic.run(x, y, z);
        sleep(duration);
        robot.holonomic.stop();
    }

    private class IterableMenu {
        Telemetry telemetry;
        ItemSet<Position> position = new ItemSet<>("Starting Position", Position.LEFT);
        ItemSet<Integer> startDelay = new ItemSet<>("Delay Before Sampling", 0);
        ItemSet<Boolean> parkInCrater = new ItemSet<>("Park in Crater", true);
        ItemSet<Boolean> dropTeamMarker = new ItemSet<>("Drop Team Marker (if on right)", true);
        ItemSet<Boolean> land = new ItemSet<>("Land BaseRobot", true);
        ItemSet<Boolean> useVuforia = new ItemSet<>("Use Vuforia/TFOD", false);

        ItemSet[] items = {position, startDelay, parkInCrater, dropTeamMarker, land,useVuforia};
        int itemsListPosition = 0;

        public IterableMenu(Telemetry telemetry) {
            this.telemetry = telemetry;
            update();
        }

        public void nextItem() {
            if (itemsListPosition < items.length) {
                itemsListPosition++;
                update();
            }
        }

        public void previousItem() {
            if (itemsListPosition > 0) {
                itemsListPosition--;
                update();
            }
        }

        public void iterateValueForward() {
            ItemSet item = items[itemsListPosition];
            if (item == position) {
                if (position.getValue() == Position.LEFT) {
                    position.setValue(Position.RIGHT);
                    position.setHasNext(false);
                    position.setHasPrevious(true);
                }
            }
            else if (item == startDelay) {
                if (startDelay.getValue() < 15) {
                    startDelay.setValue(startDelay.getValue() + 1);
                    if (startDelay.getValue() == 15) {
                        startDelay.setHasNext(false);
                        startDelay.setHasPrevious(true);
                    } else {
                        startDelay.setHasNext(true);
                        startDelay.setHasPrevious(true);
                    }
                }
            }
            else if (item == parkInCrater) {
                if (parkInCrater.getValue() == false) {
                    parkInCrater.setValue(true);
                    parkInCrater.setHasNext(false);
                    parkInCrater.setHasPrevious(true);
                }
            }
            else if (item == dropTeamMarker) {
                if (dropTeamMarker.getValue() == false) {
                    dropTeamMarker.setValue(true);
                    dropTeamMarker.setHasNext(false);
                    dropTeamMarker.setHasPrevious(true);
                }
            }
            else if (item == useVuforia) {
                if (useVuforia.getValue() == false) {
                    useVuforia.setValue(true);
                    useVuforia.setHasNext(false);
                    useVuforia.setHasPrevious(true);
                }
            }
            else if (item == land) {
                if (land.getValue() == false) {
                    land.setValue(true);
                    land.setHasNext(false);
                    land.setHasPrevious(true);
                }
            }
            update();
        }

        public void iterateValueBackwards() {
            ItemSet item = items[itemsListPosition];
            if (item == position) {
                if (position.getValue() == Position.RIGHT) {
                    position.setValue(Position.LEFT);
                    position.setHasNext(true);
                    position.setHasPrevious(false);
                }
            }
            else if (item == startDelay) {
                if (startDelay.getValue() > 0) {
                    startDelay.setValue(startDelay.getValue() - 1);
                    if (startDelay.getValue() == 0) {
                        startDelay.setHasNext(true);
                        startDelay.setHasPrevious(false);
                    } else {
                        startDelay.setHasNext(true);
                        startDelay.setHasPrevious(true);
                    }
                }
            }
            else if (item == parkInCrater) {
                if (parkInCrater.getValue() == true) {
                    parkInCrater.setValue(false);
                    parkInCrater.setHasNext(true);
                    parkInCrater.setHasPrevious(false);
                }
            }
            else if (item == dropTeamMarker) {
                if (dropTeamMarker.getValue() == true) {
                    dropTeamMarker.setValue(false);
                    dropTeamMarker.setHasNext(true);
                    dropTeamMarker.setHasPrevious(false);
                }
            }
            else if (item == useVuforia) {
                if (useVuforia.getValue() == true) {
                    useVuforia.setValue(false);
                    useVuforia.setHasNext(true);
                    useVuforia.setHasPrevious(false);
                }
            }
            else if (item == land) {
                if (land.getValue() == true) {
                    land.setValue(false);
                    land.setHasNext(true);
                    land.setHasPrevious(false);
                }
            }
            update();
        }

        public void update() {
            updateBoolean(parkInCrater);
            updateBoolean(dropTeamMarker);
            updateBoolean(useVuforia);
            updateBoolean(land);
            for (int i = 0; i<items.length; i++) {
                ItemSet itemSet = items[i];
                telemetry.addData(((i==itemsListPosition)?"--- ":"") + itemSet.getDescriptor(),
                        (itemSet.hasPrevious()?" << ":"") + itemSet.getValue() + (itemSet.hasNext()?" >> ":""));
            }
            telemetry.update();
        }

        private void updateBoolean(ItemSet<Boolean> itemSet) {
            if (itemSet.getValue()) {
                itemSet.setHasNext(false);
                itemSet.setHasPrevious(true);
            } else {
                itemSet.setHasNext(true);
                itemSet.setHasPrevious(false);
            }
        }

        private class ItemSet<T> {
            private String descriptor;
            private T value;
            private boolean hasNext = true;
            private boolean hasPrevious = false;

            public ItemSet(String descriptor, T value) {
                this.descriptor = descriptor;
                this.value = value;
            }

            public String getDescriptor() {
                return descriptor;
            }

            public T getValue() {
                return value;
            }

            public void setValue(T value) {
                this.value = value;
            }

            public boolean hasNext() {
                return hasNext;
            }

            public void setHasNext(boolean hasNext) {
                this.hasNext = hasNext;
            }

            public boolean hasPrevious() {
                return hasPrevious;
            }

            public void setHasPrevious(boolean hasPrevious) {
                this.hasPrevious = hasPrevious;
            }
        }



    }
}
