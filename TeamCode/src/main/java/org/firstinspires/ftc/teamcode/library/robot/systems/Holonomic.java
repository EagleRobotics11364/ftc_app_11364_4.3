package org.firstinspires.ftc.teamcode.library.robot.systems;

import com.qualcomm.robotcore.factory.RobotFactory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.MathOperations;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

public class Holonomic extends Drivetrain {

    private final double WHEEL_DIAMETER = 4;
    private final double WHEEL_CIRCUMFERENCE;
    private final double TICKS_PER_REVOLUTION = 288;
    private final double TICKS_PER_INCH;
    private final double DIAGONAL_BETWEEN_WHEELS = Math.sqrt(2) * 16;

    private final double ANGLE_LEFT_FRONT = 45 + 270;
    private final double ANGLE_LEFT_REAR = 135 + 270;
    private final double ANGLE_RIGHT_REAR = 225 + 270;
    private final double ANGLE_RIGHT_FRONT = 315 + 270;


    public Holonomic(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        super.frontLeftMotor = frontLeftMotor;
        super.frontRightMotor = frontRightMotor;
        super.backLeftMotor = backLeftMotor;
        super.backRightMotor = backRightMotor;

        WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER * Math.PI);
        TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
    }

    private void run(double x, double y, double z) {
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

    public void runWithoutEncoder(double x, double y, double z) {
        setMotorsMode(RUN_WITHOUT_ENCODER);
        run(x, y, z);
    }

    public void runUsingEncoderOld(double x, double y, float power) {
        double relativeAngle;
        double distancePerWheel;

        int leftFrontDistance;
        int leftRearDistance;
        int rightRearDistance;
        int rightFrontDistance;

        double leftFrontPower;
        double leftRearPower;
        double rightRearPower;
        double rightFrontPower;

        double leftFrontCos;
        double leftRearCos;
        double rightRearCos;
        double rightFrontCos;

//        x = -x;

        setMotorsMode(STOP_AND_RESET_ENCODER);

        try {
            relativeAngle = Math.toDegrees(Math.atan(y / x));
        } catch (ArithmeticException e) {
            if (y > 0) relativeAngle = 90;
            else relativeAngle = 270;
        }

        distancePerWheel = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        leftFrontCos = Math.cos(Math.toRadians(ANGLE_LEFT_FRONT - relativeAngle));
        leftRearCos = Math.cos(Math.toRadians(ANGLE_LEFT_REAR - relativeAngle));
        rightRearCos = Math.cos(Math.toRadians(ANGLE_RIGHT_REAR - relativeAngle));
        rightFrontCos = Math.cos(Math.toRadians(ANGLE_RIGHT_FRONT - relativeAngle));

        leftFrontDistance = (int) (distancePerWheel * leftFrontCos);
        leftRearDistance = (int) (distancePerWheel * leftRearCos);
        rightRearDistance = (int) (distancePerWheel * rightRearCos);
        rightFrontDistance = (int) (distancePerWheel * rightFrontCos);

        leftFrontPower = power * leftFrontCos;
        leftRearPower = power * leftRearCos;
        rightRearPower = power * rightRearCos;
        rightFrontPower = power * rightFrontCos;


        frontLeftMotor.setTargetPosition((int) (leftFrontDistance * TICKS_PER_INCH));
        backLeftMotor.setTargetPosition((int) (leftRearDistance * TICKS_PER_INCH));
        backRightMotor.setTargetPosition((int) (rightRearDistance * TICKS_PER_INCH));
        frontRightMotor.setTargetPosition((int) (rightFrontDistance * TICKS_PER_INCH));

        frontLeftMotor.setPower(leftFrontPower);
        backLeftMotor.setPower(leftRearPower);
        backRightMotor.setPower(rightRearPower);
        frontRightMotor.setPower(rightFrontPower);

        setMotorsMode(RUN_TO_POSITION);
    }

    public void runUsingEncoder(double xTarget, double yTarget, double inputPower) {
        double r;
        double theta;
        double xPrime;
        double yPrime;
        double xPower;
        double yPower;
        double LFDistanceIN;
        double LRDistanceIN;
        double RRDistanceIN;
        double RFDistanceIN;
        double LFPower;
        double LRPower;
        double RRPower;
        double RFPower;

        // set motors mode
        setMotorsMode(STOP_AND_RESET_ENCODER);

        // calculate r
        r = Math.sqrt(Math.pow(xTarget,2)+Math.pow(yTarget,2));

        // calculate theta
        if (xTarget == 0) xTarget = 0.00001;
        theta = Math.toDegrees(Math.atan(yTarget / xTarget));
        if (xTarget < 0) theta += 180;

        // calculate x and y prime
        xPrime = r * Math.cos(Math.toRadians(theta - 45));
        yPrime = r * Math.sin(Math.toRadians(theta - 45));

        // calculate x and y power
        if (yPrime > xPrime) {
            yPower = inputPower;
            xPower = inputPower * (xPrime / yPrime);
        } else {
            xPower = inputPower;
            yPower = inputPower * (yPrime / xPrime);
        }

        // set motor distances (inches)
        LFDistanceIN = xPrime;
        LRDistanceIN = yPrime;
        RRDistanceIN = -xPrime;
        RFDistanceIN = -yPrime;

        // set motor powers
        LFPower = xPower;
        LRPower = yPower;
        RRPower = -xPower;
        RFPower = -yPower;

        // program encoder targets
        frontLeftMotor.setTargetPosition((int)(LFDistanceIN * TICKS_PER_INCH));
        backLeftMotor.setTargetPosition((int)(LRDistanceIN * TICKS_PER_INCH));
        backRightMotor.setTargetPosition((int)(RRDistanceIN * TICKS_PER_INCH));
        frontRightMotor.setTargetPosition((int)(RFDistanceIN * TICKS_PER_INCH));

        // program motor power targets
        frontLeftMotor.setPower(LFPower);
        backLeftMotor.setPower(LRPower);
        backRightMotor.setPower(RRPower);
        frontRightMotor.setPower(RFPower);

        // set motors mode
        setMotorsMode(RUN_TO_POSITION);
    }

    public void turnUsingEncoder(double degrees, double power) {
        int targetPosition = (int) (degrees * TICKS_PER_REVOLUTION * (DIAGONAL_BETWEEN_WHEELS / (360 * WHEEL_DIAMETER)));
        setMotorsMode(STOP_AND_RESET_ENCODER);
        frontLeftMotor.setTargetPosition(targetPosition);
        frontRightMotor.setTargetPosition(targetPosition);
        backLeftMotor.setTargetPosition(targetPosition);
        backRightMotor.setTargetPosition(targetPosition);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public boolean motorsAreBusy() {
        if (frontLeftMotor.isBusy() | frontLeftMotor.isBusy() | backLeftMotor.isBusy() | backRightMotor.isBusy())
            return true;
        else return false;
    }


    public void setMotorsMode(DcMotor.RunMode runMode) {
        frontLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
    }

}
