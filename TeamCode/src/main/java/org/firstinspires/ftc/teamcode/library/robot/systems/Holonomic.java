package org.firstinspires.ftc.teamcode.library.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.MathOperations;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

public class Holonomic extends Drivetrain {

    public final double WHEEL_DIAMETER = 4;
    public final double TICKS_PER_REVOLUTION = 288;
    public final double INCHES_PER_TICK;

    public Holonomic(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        super.frontLeftMotor = frontLeftMotor;
        super.frontRightMotor = frontRightMotor;
        super.backLeftMotor = backLeftMotor;
        super.backRightMotor = backRightMotor;

        INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_REVOLUTION;
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

    public void runUsingEncoder(int xDistance, int yDistance, double power, DistanceUnit distanceUnit) {

        int xEncoder = (int)(xDistance / INCHES_PER_TICK);
        int yEncoder = (int)(yDistance / INCHES_PER_TICK);

        double xPower = power;
        double yPower = power;

        if (xDistance > yDistance) {
            xPower = power;
            yPower = (yDistance / xDistance) * power;
        } else if (yDistance > xDistance) {
            xPower = (xDistance / yDistance) * power;
            yPower = power;
        }

        setMotorsMode(STOP_AND_RESET_ENCODER);

        //set encoder targets
        frontLeftMotor.setTargetPosition(xEncoder);
        frontRightMotor.setTargetPosition(yEncoder);
        backLeftMotor.setTargetPosition(-yEncoder);
        backRightMotor.setTargetPosition(-xEncoder);

        setMotorsMode(RUN_TO_POSITION);
        
        frontLeftMotor.setPower(xPower);
        frontRightMotor.setPower(yPower);
        backLeftMotor.setPower(-yPower);
        backRightMotor.setPower(-xPower);

        /*final double MULTIPLY_FACTOR = Math.cos((1/(Math.PI * 4)));
        double xPower;
        double yPower;
        double xEncoderTarget;
        double yEncoderTarget;
        if (xDistance > yDistance) {
            xPower = power;
            yPower = power * (xDistance/yDistance);
        } else if (xDistance < yDistance) {
            yPower = power;
            xPower = power * (xDistance/yDistance);
        } else {
            xPower = yPower = power;
        }

        setMotorsMode(STOP_AND_RESET_ENCODER);

        xEncoderTarget = xDistance * MULTIPLY_FACTOR;
        yEncoderTarget = yDistance * MULTIPLY_FACTOR;

        int leftFrontTarget = xDistance + yDistance;
        int leftRearTarget = xDistance - yDistance;
        int rightFrontTarget = -xDistance + yDistance;
        int rightRearTarget = -xDistance - yDistance;

        setMotorsMode(RUN_TO_POSITION);

        frontLeftMotor.setTargetPosition(leftFrontTarget);
        backLeftMotor.setTargetPosition(leftRearTarget);
        frontLeftMotor.setTargetPosition(rightFrontTarget);
        backRightMotor.setTargetPosition(rightRearTarget);

        run(xPower, yPower, 0);*/
    }

    public void runUsingEncoderNormal(int xDistance, int yDistance, double power, DistanceUnit distanceUnit) {

        int xEncoder = (int)(xDistance / Math.sqrt(2));
        int yEncoder = (int)(yDistance / Math.sqrt(2));

        double xPower = power;
        double yPower = power;

        int xFrontLeft = (xEncoder + yEncoder);
        int xFrontRight = (-xEncoder + yEncoder);
        
        if (xFrontLeft > xFrontRight) {
            xPower = power;
            yPower = (xFrontRight / xFrontLeft) * power;
        } else if (xFrontLeft > xFrontRight) {
            xPower = (xFrontRight / xFrontLeft) * power;
            yPower = power;
        }

        setMotorsMode(STOP_AND_RESET_ENCODER);

        //set encoder targets
        frontLeftMotor.setTargetPosition(xEncoder + yEncoder);
        frontRightMotor.setTargetPosition(-xEncoder + yEncoder);
        backLeftMotor.setTargetPosition(xEncoder + -yEncoder);
        backRightMotor.setTargetPosition(-xEncoder + -yEncoder);

        setMotorsMode(RUN_TO_POSITION);
        frontLeftMotor.setPower(xPower);
        frontRightMotor.setPower(yPower);
        backLeftMotor.setPower(-yPower);
        backRightMotor.setPower(-xPower);
    }

        public void setMotorsMode(DcMotor.RunMode runMode) {
        frontLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
    }

}
