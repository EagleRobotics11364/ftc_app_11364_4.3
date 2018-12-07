package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;

@Autonomous(name="Encoder Test OpMode", group = "Test")
//@Disabled
public class EncoderTestOpMode extends LinearOpMode {
    BaseRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BaseRobot(hardwareMap);
        waitForStart();
        robot.holonomic.runUsingEncoder(0, 40, 0.3f);
        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
        robot.holonomic.stop();
        robot.holonomic.runUsingEncoder(40, 0, 0.3f);
        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
        robot.holonomic.stop();
    }
}
