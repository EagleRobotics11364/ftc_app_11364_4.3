package org.firstinspires.ftc.teamcode.library.systems;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Drivetrain {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    public abstract void run(double x, double z);



}
