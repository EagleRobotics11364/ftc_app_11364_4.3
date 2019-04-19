package org.firstinspires.ftc.teamcode.library.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.library.robot.systems.Holonomic;
import org.firstinspires.ftc.teamcode.library.robot.systems.HangingScrew;
import org.firstinspires.ftc.teamcode.library.robot.systems.IntakeArmPivot;
import org.firstinspires.ftc.teamcode.library.robot.systems.rgb.BlinkinController;


public class BaseRobot {
    public static final double TEAM_MARKER_RETRACTED = 0.30;
    public static final double TEAM_MARKER_RELEASED = 0.07;

    public static final double INTAKE_SILVER_OUTPUT_LEFT = 0.60;
    public static final double INTAKE_SILVER_OUTPUT_CENTER = 0.76;
    public static final double INTAKE_SILVER_OUTPUT_RIGHT = 0.92;

    public static final double INTAKE_BOX_POSITION_COLLECTION_OR_DEPOSIT = 0.18
            ;
    public static final double INTAKE_BOX_POSITION_HOLDING = 0.34;

    // Drivetrain (DcMotor) Variables (Section 1)
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    // Tape Measure Spool (DcMotor) Variables (Section 2)
    private DcMotor hangingScrewMotor;

    // Servo Variables (Section 3)
    public Servo teamMarkerServo;

    // Color/Distance Sensor Variables (Section 4)
    public ColorSensor leftColorSensor;
    public ColorSensor rightColorSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    // Intake Variables (Section 5)
    private DcMotor intakeArmPivotMotor;
    public DcMotor collectorMotor;
    public DcMotor intakeArmExtensionMotor;
    public Servo collectionBoxPivotServo;
    public Servo outputDirectionSwitcherServo;

    // IMU Variables (Section 6)
    public BNO055IMU imuA;
    public BNO055IMU imuB;

    // Analog Input Variables (Section 7)
    public AnalogInput potentiometer;

    // Robot Systems Variables (Section 8)
    public Holonomic holonomic;
    public HangingScrew hangingScrew;
    public IntakeArmPivot intakeArmPivotController;

    // REV Blinkin LED Variables (Section 9)
    public RevBlinkinLedDriver blinkin;

    public BaseRobot(HardwareMap hardwareMap) {
        // Motors (Section 1)
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Tape Measure Spool (DcMotor) Variables (Section 2)
        hangingScrewMotor = hardwareMap.dcMotor.get("hangingScrew");

        // Servo Variables (Section 3)
        teamMarkerServo = hardwareMap.servo.get("teamMarkerServo");

        // Color/Distance Sensor Variables (Section 4)
        leftColorSensor = hardwareMap.colorSensor.get("leftColorSensor");
        rightColorSensor = hardwareMap.colorSensor.get("rightColorSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightColorSensor");

        // Intake Variables (Section 5)
        intakeArmPivotMotor = hardwareMap.dcMotor.get("intakeArmPivotMotor");
        intakeArmExtensionMotor = hardwareMap.dcMotor.get("intakeArmExtensionMotor");
        collectionBoxPivotServo = hardwareMap.servo.get("collectionBoxPivotServo");
        outputDirectionSwitcherServo = hardwareMap.servo.get("outputDirectionSwitcherServo");
        collectorMotor = hardwareMap.dcMotor.get("collectorMotor");

        // IMU Variables (Section 6)
        imuA = hardwareMap.get(BNO055IMU.class, "imuA");
        imuB = hardwareMap.get(BNO055IMU.class, "imuB");

        // Analog Input Variables (Section 7)
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        // Robot Systems Variables (Section 8)
        holonomic = new Holonomic(frontLeftMotor, backLeftMotor,frontRightMotor,backRightMotor);
        hangingScrew = new HangingScrew(hangingScrewMotor);
        intakeArmPivotController = new IntakeArmPivot(intakeArmPivotMotor, potentiometer);

        // REV Blinkin LED Variables (Section 9)
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


    }



}
