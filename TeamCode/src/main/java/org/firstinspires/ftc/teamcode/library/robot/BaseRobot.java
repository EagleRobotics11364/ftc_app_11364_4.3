package org.firstinspires.ftc.teamcode.library.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.robot.systems.DualTapeSpools;
import org.firstinspires.ftc.teamcode.library.robot.systems.Holonomic;


public class BaseRobot {

    // Drivetrain (DcMotor) Variables
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    // Tape Measure Spool (DcMotor) Variables
    private DcMotor frontTapeMeasure;
    private DcMotor backTapeMeasure;

    // Intake Variables
    public DcMotor intakeArmMotor;
    public Servo intakeBallServo;
    public Servo intakeArmHoldServo;

    // Servo Variables
    public Servo teamMarkerServo;
    public Servo craterArm;

    // Color/Distance Sensor Variables
    public ColorSensor leftColorSensor;
    public ColorSensor rightColorSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    // BaseRobot Systems Variables
    public Holonomic holonomic;
    public DualTapeSpools dualTapeSpools;

    public BaseRobot(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontTapeMeasure = hardwareMap.dcMotor.get("frontTapeMeasure");
        backTapeMeasure = hardwareMap.dcMotor.get("backTapeMeasure");

        intakeArmMotor = hardwareMap.dcMotor.get("intakeArmMotor");
        intakeBallServo = hardwareMap.servo.get("intakeBallServo");
        intakeArmHoldServo = hardwareMap.servo.get("intakeArmHoldServo");

        teamMarkerServo = hardwareMap.servo.get("teamMarkerServo");
        craterArm = hardwareMap.servo.get("craterArm");

        leftColorSensor = hardwareMap.colorSensor.get("leftColorSensor");
        rightColorSensor = hardwareMap.colorSensor.get("rightColorSensor");

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightColorSensor");

        holonomic = new Holonomic(frontLeftMotor, backLeftMotor,frontRightMotor,backRightMotor);
        dualTapeSpools = new DualTapeSpools(frontTapeMeasure, backTapeMeasure);
    }



}
