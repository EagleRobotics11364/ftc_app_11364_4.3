package org.firstinspires.ftc.teamcode.library.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip;

public class DualTapeSpools implements Stoppable{
    DcMotor frontTapeSpool;
    DcMotor backTapeSpool;

    public DualTapeSpools(DcMotor frontTapeSpool, DcMotor backTapeSpool) {
        this.frontTapeSpool = frontTapeSpool;
        this.backTapeSpool = backTapeSpool;
    }

    public void move(double frontPower, double backPower) {
        frontTapeSpool.setPower(-rangeClip(frontPower));
        backTapeSpool.setPower(rangeClip(backPower));
    }

    public void move(double proportionalPower) {
        proportionalPower = rangeClip(proportionalPower);
        move(0.67 * proportionalPower, proportionalPower);
    }

    @Override
    public void stop() {
        frontTapeSpool.setPower(0);
        backTapeSpool.setPower(0);

    }
}
