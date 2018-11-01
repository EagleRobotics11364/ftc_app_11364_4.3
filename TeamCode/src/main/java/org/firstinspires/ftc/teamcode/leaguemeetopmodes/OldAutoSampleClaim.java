package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.leaguemeetopmodes.LeagueMeet1Robot.SERVO_POSITION_DOWN;
import static org.firstinspires.ftc.teamcode.leaguemeetopmodes.LeagueMeet1Robot.SERVO_POSITION_UP;


@Autonomous(name="Claim and Sample: NO LANDING", group="League Meets")
@Disabled
public class OldAutoSampleClaim extends LinearOpMode {

    private LeagueMeet1Robot robot;
    private ElapsedTime elapsedTime = new ElapsedTime();

    private TeamColor teamColor = TeamColor.NULL;
    private StartingPosition startingPosition = StartingPosition.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize robot
        robot = new LeagueMeet1Robot(hardwareMap);
        robot.teamMarkerServo.setPosition(SERVO_POSITION_UP);

        while(!isStarted()) changeRobotStartPosition();

        // after op mode starts:
//        robot.tapeMeasureHook.proportionalMove(1.0f);
//        sleep(17000);
//        robot.tapeMeasureHook.proportionalMoveDown(0f);

//        drive(-0.3, 0.0, 0.0, 500); //drive out of lander hook
//        drive(0.0, 1, 0.0, 1500); //drive to depot
        drive(0.0, 0.60, 0.0, 1000);
        driveToSample();
        { // container for extra variables
            final double driveBackPower = 0.4;
            final long driveBackTime = 500;
            sleep(500);
            if (sampleIsGold()) {
                pushSample();
                drive(0.0, 0.5, 0.0, 900);
            }
            else { //drive to the right sample
                drive(0.0, -driveBackPower, 0.0, driveBackTime);
                sleep(500);
                drive(0.7, 0.0, 0.0, 800);
                sleep(500);
                drive(0.0, driveBackPower, 0.0, driveBackTime);
                sleep(500);
                driveToSample();
                if (sampleIsGold()) {
                    pushSample();
//                    drive(0.0, -driveBackPower, 0.0, driveBackTime);
//                    drive(-0.5, 0.0, 0.0, 885);
                    drive(0.0, 0.5, 0.0, 600);
                    drive(0.0, 0.0, -0.5, 500);
                    drive(0.2, 0.2, 0.0, 300);

                } else { //drive to the left sample
                    drive(0.0, -driveBackPower, 0.0, driveBackTime);
                    drive(-0.7, 0.0, 0.0, 1600);
                    driveToSample();
                    pushSample();
//                    drive(0.0, -driveBackPower, 0.0, driveBackTime);
//                    drive(0.5, 0.0, 0.0, 885);
                    drive(0.10, 0.7, 0.0, 1200);
                }
            }
        }
        robot.teamMarkerServo.setPosition(SERVO_POSITION_DOWN);
        sleep(1000);


    }

    private void drive(double x, double y, double z) {
        robot.holonomic.run(x, -y, z);
        robot.holonomic.stop();
    }

    private void drive(double x, double y, double z, long msTime) {
        robot.holonomic.run(x, -y, z);
        sleep(msTime);
        robot.holonomic.stop();
    }

    private void driveToSample() {
        robot.holonomic.run(0.0, -0.4, 0.0);
        while(getHueFromColorSensor() > 100)
        {

            telemetry.addData("alpha", robot.colorSensor.alpha());
            telemetry.update();
        }
        robot.holonomic.stop();
    }

    private void changeRobotStartPosition() {
         if (gamepad1.x) teamColor = TeamColor.BLUE;
    else if (gamepad1.b) teamColor = TeamColor.RED;

         if (gamepad1.dpad_left)  startingPosition = StartingPosition.LEFT;
    else if (gamepad1.dpad_right) startingPosition = StartingPosition.RIGHT;

        telemetry.addData("Alliance Color", teamColor);
        telemetry.addData("Starting Position", startingPosition);
        telemetry.update();
    }

    private void pushSample() {
        drive(0, 0, -0.8, 300);
        sleep(200);
        drive(0, 0, 0.8, 300);
    }

    private SampleColor getSampleColor() {
        SampleColor sampleColor = SampleColor.NULL;
        int inputAlpha = robot.colorSensor.alpha();

              if (inputAlpha > 95) sampleColor = SampleColor.WHITE;
         else if (inputAlpha >  60) sampleColor = SampleColor.YELLOW;

         return sampleColor;
    }

    private boolean sampleIsGold() {
        int inputAlpha = robot.colorSensor.alpha();

        telemetry.addData("read alpha", inputAlpha);
        telemetry.addData("read hue", getHueFromColorSensor());
        telemetry.update();
        if (inputAlpha >= 60 & inputAlpha <= 140 & getHueFromColorSensor() < 40) return true;
        else return false;
    }

    private float getHueFromColorSensor() {
        float[] HSV = {0F,0F,0F};
        Color.RGBToHSV( (int) robot.colorSensor.red() * 255,
                (int) robot.colorSensor.green() * 255,
                (int) robot.colorSensor.blue() * 255,
                HSV);
        return HSV[0];
    }
    enum TeamColor {
        BLUE, RED, NULL
    }
    enum SampleColor {
        WHITE, YELLOW, NULL
    }
    enum StartingPosition {
        LEFT, RIGHT, NULL
    }
}
