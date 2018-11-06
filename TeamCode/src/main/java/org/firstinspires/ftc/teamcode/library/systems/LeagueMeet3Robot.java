package org.firstinspires.ftc.teamcode.library.systems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class LeagueMeet3Robot {

    // Drivetrain (DcMotor) Variables
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    // Tape Measure Spool (DcMotor) Variables
    private DcMotor frontTapeMeasure;
    private DcMotor backTapeMeasure;

    // Servo Variables
    public Servo teamMarkerServo;

    // Color/Distance Sensor Variables
    public ColorSensor leftColorSensor;
    public ColorSensor rightColorSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    // Robot Systems Variables
    public Holonomic holonomic;
    public DualTapeSpools dualTapeSpools;

    public LeagueMeet3Robot(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontTapeMeasure = hardwareMap.dcMotor.get("frontTapeMeasure");
        backTapeMeasure = hardwareMap.dcMotor.get("backTapeMeasure");

        teamMarkerServo = hardwareMap.servo.get("teamMarkerServo");

        leftColorSensor = hardwareMap.colorSensor.get("leftColorSensor");
        rightColorSensor = hardwareMap.colorSensor.get("rightColorSensor");

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightColorSensor");

        holonomic = new Holonomic(frontLeftMotor, backLeftMotor,frontRightMotor,backRightMotor);
        dualTapeSpools = new DualTapeSpools(frontTapeMeasure, backTapeMeasure);
    }



}
