package org.firstinspires.ftc.teamcode.leaguemeetopmodes.meet3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.ColorOperations;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.sampling.FieldSample;
import org.firstinspires.ftc.teamcode.library.sampling.VuforiaTFODSampler;
import org.firstinspires.ftc.teamcode.library.systems.LeagueMeet3Robot;


@Autonomous(name = "League Meet 3 Auto", group = "Meet3")
public class LeagueMeet3AutoMenu extends LinearOpMode {

    // robot, core systems variables
    private LeagueMeet3Robot robot;
    private VuforiaTFODSampler vuforiaTFODSampler;

    // action variables
    private Position startingPosition = Position.NULL;
    private boolean useVuforiaTFOD = false;
    private boolean parkInCrater = false;
    private boolean dropTeamMarker = false;
    private int secondsDelay = 0;

    // learned variables
    private Position goldSamplePosition = Position.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        // init robot
        robot = new LeagueMeet3Robot(hardwareMap);
        robot.teamMarkerServo.setPosition(0);
        IterableMenu iterableMenu = new IterableMenu(telemetry);
        while (!isStarted() & opModeIsActive()) {
            if (gamepad1.dpad_up) {
                iterableMenu.previousItem();
                while (gamepad1.dpad_up & opModeIsActive()) ;
            } else if (gamepad1.dpad_down) {
                iterableMenu.nextItem();
                while (gamepad1.dpad_down & opModeIsActive()) ;
            } else if (gamepad1.dpad_left) {
                iterableMenu.iterateValueBackwards();
                while (gamepad1.dpad_left & opModeIsActive()) ;
            } else if (gamepad1.dpad_right) {
                iterableMenu.iterateValueForward();
                while (gamepad1.dpad_right & opModeIsActive()) ;
            }
        }

        startingPosition = iterableMenu.position.getValue();
        useVuforiaTFOD = iterableMenu.useVuforia.getValue();
        secondsDelay = iterableMenu.startDelay.getValue();
        parkInCrater = iterableMenu.parkInCrater.getValue();
        dropTeamMarker = iterableMenu.dropTeamMarker.getValue();

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
                    drive(-0.2, 0.6, 0, 1);
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

    private class IterableMenu {
        Telemetry telemetry;
        ItemSet<Position> position = new ItemSet<>("Starting Position", Position.LEFT);
        ItemSet<Integer> startDelay = new ItemSet<>("Delay Before Sampling", 0);
        ItemSet<Boolean> parkInCrater = new ItemSet<>("Park in Crater", true);
        ItemSet<Boolean> dropTeamMarker = new ItemSet<>("Drop Team Marker", true);
        ItemSet<Boolean> useVuforia = new ItemSet<>("Use Vuforia/TFOD", false);

        ItemSet[] items = {position, startDelay, parkInCrater, dropTeamMarker, useVuforia};
        int itemsListPosition = 0;

        public IterableMenu(Telemetry telemetry) {
            this.telemetry = telemetry;
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
            update();
        }

        public void update() {
            for (int i = 0; i<items.length; i++) {
                ItemSet itemSet = items[i];
                telemetry.addData((i==itemsListPosition)?"--- ":""+ itemSet.getDescriptor(),
                        (itemSet.hasPrevious()?" << ":"") + itemSet.getValue() + (itemSet.hasNext()?" >> ":""));
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
