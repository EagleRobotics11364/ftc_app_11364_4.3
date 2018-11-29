package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;
import org.firstinspires.ftc.teamcode.library.robot.systems.IMUDrivetrainController;

@Autonomous(name="Encoder Test OpMode", group = "Test")
//@Disabled
public class EncoderTestOpMode extends LinearOpMode {
    BaseRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BaseRobot(hardwareMap);
        robot.holonomic.runUsingEncoder(12, 24, 0.5, DistanceUnit.INCH);
        sleep(1000);
    }
}
