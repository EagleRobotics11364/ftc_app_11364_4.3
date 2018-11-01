package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.leaguemeetopmodes.LeagueMeet1Robot.SERVO_POSITION_DOWN;
import static org.firstinspires.ftc.teamcode.leaguemeetopmodes.LeagueMeet1Robot.SERVO_POSITION_UP;


@Autonomous(name="Claim Depot Only: NO LANDING", group="League Meets")
//@Disabled
public class OldAutoClaimOnly extends LinearOpMode {

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
        drive(0.0, 1, 0.0, 1500); //drive to depot

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
        float[] HSV = {0F,0F,0F};
        Color.RGBToHSV( (int) robot.colorSensor.red() * 255,
                (int) robot.colorSensor.green() * 255,
                (int) robot.colorSensor.blue() * 255,
                HSV);
        telemetry.addData("read alpha", inputAlpha);
        telemetry.addData("read hue", HSV[0]);
        telemetry.update();
        if (inputAlpha >= 60 & inputAlpha <= 140 & HSV[0] < 40) return true;
        else return false;
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
