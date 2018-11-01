package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.systems.drivetrain.Holonomic;
import org.firstinspires.ftc.teamcode.library.systems.TapeMeasureHook;

public class LeagueMeet1Robot {

    public static final float SERVO_POSITION_UP = 0.8f;
    public static final float SERVO_POSITION_DOWN = 0.3f;

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    public DcMotor frontTapeMeasure;
    public DcMotor backTapeMeasure;

    public Servo teamMarkerServo;

    public ColorSensor colorSensor;

    public Holonomic holonomic;

    public TapeMeasureHook tapeMeasureHook;

    public LeagueMeet1Robot(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontTapeMeasure = hardwareMap.dcMotor.get("frontTapeMeasure");
        backTapeMeasure = hardwareMap.dcMotor.get("backTapeMeasure");

        teamMarkerServo = hardwareMap.servo.get("teamMarkerServo");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        holonomic = new Holonomic(frontLeftMotor, backLeftMotor,frontRightMotor,backRightMotor);

        tapeMeasureHook = new TapeMeasureHook(frontTapeMeasure, backTapeMeasure);
    }

    public void stopAllMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontTapeMeasure.setPower(0);
        backTapeMeasure.setPower(0);
    }
}
