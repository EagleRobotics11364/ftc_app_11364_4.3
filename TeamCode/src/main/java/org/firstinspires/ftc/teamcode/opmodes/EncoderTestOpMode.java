package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;

@Autonomous(name="Encoder Test OpMode", group = "Test")
//@Disabled
public class EncoderTestOpMode extends LinearOpMode {
    BaseRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BaseRobot(hardwareMap);
        waitForStart();
//        robot.holonomic.runUsingEncoder(0, 10, 0.5f);
//        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
//        while(!gamepad1.a & opModeIsActive());
//        robot.holonomic.stop();
//        robot.holonomic.runUsingEncoder(10, 0, 0.5f);
//        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
//        while(!gamepad1.a & opModeIsActive());
//        robot.holonomic.stop();
//        robot.holonomic.runUsingEncoder(0, -10, 0.5f);
//        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
//        while(!gamepad1.a & opModeIsActive());
//        robot.holonomic.stop();
//        robot.holonomic.runUsingEncoder(-10, 0, 0.5f);
//        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
//        while(!gamepad1.a & opModeIsActive());
//        robot.holonomic.stop();
//
//        robot.holonomic.runUsingEncoder(10, 10, 0.5f); // front right
//        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
//        while(!gamepad1.a & opModeIsActive());
//        robot.holonomic.stop();
//        robot.holonomic.runUsingEncoder(-10, 10, 0.5f); // front left
//        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
//        while(!gamepad1.a & opModeIsActive());
//        robot.holonomic.stop();
//        robot.holonomic.runUsingEncoder(20, 10, 0.8); // back left
//        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
//        robot.holonomic.stop();
//        while(!gamepad1.a & opModeIsActive());
        robot.holonomic.runUsingEncoder(-13, 10, 0.5f); // back right
        while(robot.holonomic.motorsAreBusy() & opModeIsActive());
        while(!gamepad1.a & opModeIsActive());
        robot.holonomic.stop();
    }
}
