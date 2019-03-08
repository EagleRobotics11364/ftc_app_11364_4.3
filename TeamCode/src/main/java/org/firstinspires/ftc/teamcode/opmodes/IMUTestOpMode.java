package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.IterableTelemetryMenu;
import org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.MenuItemInteger;
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;
import org.firstinspires.ftc.teamcode.library.robot.systems.IMUDrivetrainController;

@Autonomous(name="IMU Test OpMode", group = "Test")
//@Disabled
public class IMUTestOpMode extends LinearOpMode {
    BaseRobot robot;
    IMUDrivetrainController imuController;
    IterableTelemetryMenu menu;
    Integer integer = Integer.valueOf(0);
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BaseRobot(hardwareMap);
        imuController = new IMUDrivetrainController(hardwareMap.get(BNO055IMU.class, "imuA"), robot.holonomic);
        menu = new IterableTelemetryMenu(telemetry);
        menu.add(new MenuItemInteger("int", "turn",0,-360,360));
        while(!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_right) menu.iterateForward();
            else if (gamepad1.dpad_left) menu.iterateBackward();
            else if (gamepad1.right_bumper) for (int i= 0; i<10;i++) menu.iterateForward();
            else if (gamepad1.left_bumper) for (int i = 0; i<10; i++) menu.iterateBackward();
            while ((gamepad1.dpad_right | gamepad1.dpad_left | gamepad1.right_bumper | gamepad1.left_bumper)&&!isStopRequested());

        }
        if (!isStopRequested()) {
            imuController.rotateForDegrees(integer, 0.5f);
        }
    }
}
