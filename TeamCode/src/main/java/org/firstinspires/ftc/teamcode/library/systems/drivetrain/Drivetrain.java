package org.firstinspires.ftc.teamcode.library.systems.drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Kk4jr on 9/23/2017.
 *
 * This is an abstract class for 4 wheel robot drivetrains
 *
 */

public abstract class Drivetrain {
    DcMotor leftFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightFrontMotor;
    DcMotor rightRearMotor;

    /**
     *
     * @param leftFrontMotor
     * @param leftRearMotor
     * @param rightFrontMotor
     * @param rightRearMotor
     */
    public Drivetrain(DcMotor leftFrontMotor, DcMotor leftRearMotor, DcMotor rightFrontMotor, DcMotor rightRearMotor){
        this.leftFrontMotor = leftFrontMotor;
        this.leftRearMotor = leftRearMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightRearMotor = rightRearMotor;
    }

    /**
     *
     * @param x Forward/Reverse Motion of the Robot + is Forward - is Reverse
     * @param z Rotaion of the Robot + is Clockwise - is Counter-Clockwise
     */
    public abstract void run(double x, double z);

}