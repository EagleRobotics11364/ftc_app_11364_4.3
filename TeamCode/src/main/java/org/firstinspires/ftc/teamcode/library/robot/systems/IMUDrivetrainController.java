package org.firstinspires.ftc.teamcode.library.robot.systems;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMUDrivetrainController {
    private BNO055IMU imuA;
//    private BNO055IMU imuB;

    private Orientation imuAOriginalOrientation;
//    private Orientation imuBOriginalOrientation;

    private Drivetrain drivetrain;


    public IMUDrivetrainController(BNO055IMU imuA, Drivetrain drivetrain) {
        this.imuA = imuA;
        this.drivetrain = drivetrain;
    }

    public void rotateForDegrees(float degrees, float power) {
        float targetAngle = getCurrentIMUOrientation().firstAngle + degrees;
        float currentAngle = targetAngle;
        double finalPower = power;
        drivetrain.run(0, finalPower);
        while (true) {
            currentAngle = getCurrentIMUOrientation().firstAngle;
            if (currentAngle/targetAngle < 0.50) {
                finalPower = power * (currentAngle/targetAngle);
                if (finalPower < 0.10 && finalPower >= 0) {
                    finalPower = 0.10;
                } else if (finalPower > -0.10 && finalPower <= 0) {
                    finalPower = 0.10;
                }
                drivetrain.run(0,finalPower);
            }
        }
    }

    public Orientation getCurrentIMUOrientation() {
        return imuA.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
