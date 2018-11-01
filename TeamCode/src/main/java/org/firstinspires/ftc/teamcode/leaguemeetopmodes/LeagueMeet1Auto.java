package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

@Autonomous(name="Auto: Land and Claim", group="League Meets")
@Disabled
public class LeagueMeet1Auto extends LinearOpMode {
    public static final float SERVO_POSITION_UP = 0.8f;
    public static final float SERVO_POSITION_DOWN = 0.3f;

    LeagueMeet1Robot robot;

    @Override
    public void runOpMode() {
        robot = new LeagueMeet1Robot(hardwareMap);
        robot.teamMarkerServo.setPosition(SERVO_POSITION_UP);
        while(!opModeIsActive()) {}
        robot.tapeMeasureHook.proportionalMove(1);
        sleep(25000);
        robot.tapeMeasureHook.stop();
        robot.frontTapeMeasure.setPower(1);
        sleep(2000);
        robot.tapeMeasureHook.stop();
        drive(-0.2, 0, 0, 500);
        drive(0,1,0,1500);
        robot.teamMarkerServo.setPosition(SERVO_POSITION_DOWN);
//        robot.frontTapeMeasure.setPower(1);
//        sleep(2000);
    }

    private void drive(double x, double y, double z, long msTime) {
        robot.holonomic.run(x, -y, z);
        sleep(msTime);
        robot.holonomic.stop();
    }
}
