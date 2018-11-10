package org.firstinspires.ftc.teamcode.leaguemeetopmodes.meet3;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.sampling.VuforiaTFODSampler;
import org.firstinspires.ftc.teamcode.library.systems.LeagueMeet3Robot;

@Autonomous(name = "Autonomous Learn", group = "Meet3")
@Disabled
public class LeagueMeet3AutoLearn extends LinearOpMode {

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
//         init robot
        int objects = 0;
        robot = new LeagueMeet3Robot(hardwareMap);

        waitForStart();

        robot.holonomic.run(0.5,0,0);
        while(objects < 3) {
            float obtainedHue;
            while((obtainedHue = calculateHue(robot.rightColorSensor)) > 110) {
                telemetry.addData("Hue", obtainedHue);
                telemetry.update();
            }
            objects++;
            if(obtainedHue < 40) break;
            while((obtainedHue = calculateHue(robot.rightColorSensor)) < 110) {
                telemetry.addData("Hue", obtainedHue);
                telemetry.update();
            }
        }
        drive(0,-0.7,0,200);
        switch (objects) {
            case 1: goldSamplePosition = Position.LEFT; break;
            case 2: goldSamplePosition = Position.CENTER; break;
            case 3: goldSamplePosition = Position.RIGHT; break;
        }
        telemetry.setAutoClear(false);
        telemetry.addData("pos", goldSamplePosition);
        telemetry.update();
        robot.holonomic.stop();

//        robot.holonomic.run(0.5, 0, 0);
//
//        double leftDistance = 0;
//        double rightDistance = 0;
//        float obtainedHue = 0f;
//        for(int i = 0; i<2; i++) {
//            /* wait */ while(Double.isNaN(rightDistance = robot.rightDistanceSensor.getDistance(DistanceUnit.CM)) || rightDistance>10);
//            leftDistance = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);
//
//            // get hue from color sensor closest to an object
//            sleep(200);
//            if (rightDistance < leftDistance) obtainedHue = calculateHue(robot.rightColorSensor);
//            else obtainedHue = calculateHue(robot.leftColorSensor);
//
//            // if obtainedHue is less than 60, push the sample
//            if (obtainedHue < 60) {
//                drive(0, -0.7, 0, 200);
//                break;
//            }
//            sleep(500);
//        }
//        robot.holonomic.stop();
//        telemetry.addData("left distance", leftDistance);
//        telemetry.addData("right distance", rightDistance);
//        telemetry.addData("obtained hue", obtainedHue);
;
        telemetry.update();

        sleep(5000);

    }

    private void doVuforiaSampling() {
        try {
            goldSamplePosition = vuforiaTFODSampler.recognize();
            switch(goldSamplePosition) {
                case LEFT:
                    drive(0.2, -0.6, 0, 1);
                case CENTER:
                    drive(0,-0.5, 0, 1);
                case RIGHT:
                    drive(-0.2, -0.6, 0, 1);
                case NULL:
                    doColorSensorSampling();
            }
        } catch (NullPointerException e) {
            doColorSensorSampling();
        }

    }

    private float calculateHue(ColorSensor colorSensor) {
        return calculateHue(colorSensor.red(), colorSensor.green(), colorSensor.blue());
    }


    private float calculateHue(int red, int green, int blue) {
        final double SCALE_FACTOR = 255;
        float hsvValues[] = new float[3];
        Color.RGBToHSV((int) (red * SCALE_FACTOR),
                (int) (green * SCALE_FACTOR),
                (int) (blue * SCALE_FACTOR),
                hsvValues);
        return hsvValues[0];
    }

    private void doColorSensorSampling() {
        drive(-0.2, -0.6, 0, 1000);
    }

    private void drive(double x, double y, double z, long duration) {
        robot.holonomic.run(x, y, z);
        sleep(duration);
        robot.holonomic.stop();
    }
}
