package org.firstinspires.ftc.teamcode.testopmodes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;
import org.firstinspires.ftc.teamcode.library.robot.systems.IMUDrivetrainController;

@TeleOp(name = "Actual IMU Test", group = "IMU")
@Disabled
public class IMUActualTestOpMode extends LinearOpMode {

    private BaseRobot robot;
    BNO055IMU imuA;
    IMUDrivetrainController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMU_A_CalibrationData.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imuA = hardwareMap.get(BNO055IMU.class, "imuA");
        imuA.initialize(parameters);
        controller = new IMUDrivetrainController(imuA, robot.holonomic, telemetry);
        sleep(1000);
        controller.rotateSuper(45);
    }
}
