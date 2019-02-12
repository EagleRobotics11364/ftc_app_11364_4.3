package org.firstinspires.ftc.teamcode.library.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.robot.systems.Holonomic;
import org.firstinspires.ftc.teamcode.library.robot.systems.LiftingScrew;


public class BaseRobot {

    // Drivetrain (DcMotor) Variables
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    // Tape Measure Spool (DcMotor) Variables
    public DcMotor hangingScrew;

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

    // Robot Systems Variables
    public Holonomic holonomic;
//    public LiftingScrew liftingScrew;
//    public DualTapeSpools dualTapeSpools;

    public BaseRobot(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        hangingScrew = hardwareMap.dcMotor.get("hangingScrew");

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
//        rearLiftingScrews = new LiftingScrew(leftHangingScrew, rightHangingScrew);
//        dualTapeSpools = new DualTapeSpools(leftHangingScrew, rightHangingScrew);
    }



}
