package org.firstinspires.ftc.teamcode.library.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Drivetrain implements Stoppable {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }


}
