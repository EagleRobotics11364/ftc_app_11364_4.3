package org.firstinspires.ftc.teamcode.library.robot.systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMUDrivetrainController {
    private BNO055IMU imuA;
//    private BNO055IMU imuB;

    private Orientation imuAOriginalOrientation;
//    private Orientation imuBOriginalOrientation;

    private Holonomic drivetrain;

private Telemetry telemetry;
    public IMUDrivetrainController(BNO055IMU imuA, Holonomic drivetrain, Telemetry telemetry) {
        this.imuA = imuA;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMU_A_CalibrationData.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imuA.initialize(parameters);
        imuA.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        this.drivetrain = drivetrain;
        this.telemetry = telemetry;
    }

    public float getCurrentHeading() {
        return getCurrentIMUOrientation().firstAngle;
    }

    public float getCurrentRoll() {
        return getCurrentIMUOrientation().secondAngle;
    }

    public float getCurrentPitch() {
        return getCurrentIMUOrientation().thirdAngle;
    }

    public void _rotateForDegrees(float degrees, float power) {
        float currentAngle = getCurrentIMUOrientation().firstAngle;
        float targetAngle = currentAngle + degrees;

        double finalPower = -power;
        drivetrain.runWithoutEncoder(0, 0, finalPower);
        while (true) {
            currentAngle = getCurrentIMUOrientation().firstAngle;
            if (currentAngle/targetAngle < 0.50) {
                finalPower = power * (currentAngle/targetAngle);
                if (finalPower < 0.10 && finalPower >= 0) {
                    finalPower = -0.10;
                } else if (finalPower > -0.10 && finalPower <= 0) {
                    finalPower = 0.10;
                }
                drivetrain.runWithoutEncoder(0,0,finalPower);
            }
            telemetry.addData("current", currentAngle);
            telemetry.addData("target", targetAngle);
            telemetry.update();
        }
    }

    public void rotateSuper(int target) {
        target = -target;
        ElapsedTime time = new ElapsedTime();
        time.reset();
        time.startTime();
        float temp = getCurrentIMUOrientation().firstAngle;
        while (time.seconds() < 8) {
            temp = getCurrentIMUOrientation().firstAngle;

            if (target > 0 && temp < 0) {
                drivetrain.runWithoutEncoder(0, 0, .40 - (time.seconds() * .05));
            } else if (target < 0 && temp > 0) {
                drivetrain.runWithoutEncoder(0, 0, -.40 + (time.seconds() * .05));
            } else if (temp > target) {
                drivetrain.runWithoutEncoder(0, 0, .40 - (time.seconds() * .05));
            } else if (temp < target) {
                drivetrain.runWithoutEncoder(0, 0, -.40 + (time.seconds() * .05));
            }
        }
        drivetrain.stop();
    }

    public Orientation getCurrentIMUOrientation() {
        return imuA.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }
}
