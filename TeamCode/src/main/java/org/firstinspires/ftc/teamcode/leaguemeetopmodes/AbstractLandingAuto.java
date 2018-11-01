package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AbstractLandingAuto extends LinearOpMode {
    public static final float SERVO_POSITION_UP = 0.8f;
    public static final float SERVO_POSITION_DOWN = 0.3f;

   LeagueMeet1Robot robot;

    @Override
    public void runOpMode() {
        robot = new LeagueMeet1Robot(hardwareMap);
        servoStartPosition();
        land();
        doAfterLanding();
//        robot.frontTapeMeasure.setPower(1);
//        sleep(2000);
    }

    final void land() {
        while(!opModeIsActive()) {}
        robot.tapeMeasureHook.proportionalMove(1);
        sleep(25000);
        robot.tapeMeasureHook.stop();
        robot.frontTapeMeasure.setPower(1);
        sleep(2000);
        robot.tapeMeasureHook.stop();
    }

    abstract void doAfterLanding();

    void servoStartPosition() {
        robot.teamMarkerServo.setPosition(SERVO_POSITION_UP);
    }

    void servoDropTeamMarker() {

    }

    void drive(double x, double y, double z, long msTime) {
        robot.holonomic.run(x, -y, z);
        sleep(msTime);
        robot.holonomic.stop();
    }


}
