package org.firstinspires.ftc.teamcode.library.systems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.library.functions.MathOperations;

public class Holonomic extends Drivetrain {

    public Holonomic(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        super.frontLeftMotor = frontLeftMotor;
        super.frontRightMotor = frontRightMotor;
        super.backLeftMotor = backLeftMotor;
        super.backRightMotor = backRightMotor;
    }

    @Override
    public void run(double y, double z) {
        run(0, y, z);
    }

    public void run(double x, double y, double z) {
        x = MathOperations.rangeClip(x, -1, 1);
        y = MathOperations.rangeClip(y, -1, 1);
        z = MathOperations.rangeClip(z, -1, 1);

        double leftFrontPower = x + y + z;
        double leftRearPower = x - y + z;
        double rightFrontPower = -x + y + z;
        double rightRearPower = -x - y + z;
        frontLeftMotor.setPower(leftFrontPower);
        backLeftMotor.setPower(leftRearPower);
        frontRightMotor.setPower(rightFrontPower);
        backRightMotor.setPower(rightRearPower);

    }

}
