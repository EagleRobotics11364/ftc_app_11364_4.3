package org.firstinspires.ftc.teamcode.library.systems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TapeMeasureHook {
    private DcMotor frontTapeMeasure;
    private DcMotor backTapeMeasure;

    public TapeMeasureHook(DcMotor frontTapeMeasure, DcMotor backTapeMeasure) {
        this.frontTapeMeasure = frontTapeMeasure;
        this.backTapeMeasure = backTapeMeasure;
    }

    public void proportionalMove(float speed) {
        frontTapeMeasure.setPower(speed*0.67);
        backTapeMeasure.setPower(speed);
    }

    public void proportionalMoveUp(float speed) {
        int target = 5000;
        frontTapeMeasure.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backTapeMeasure.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontTapeMeasure.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backTapeMeasure.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontTapeMeasure.setTargetPosition(5000);
        backTapeMeasure.setTargetPosition(5000);
    }

    public void proportionalMoveDown(float speed) {

    }

    public void stop() {
        frontTapeMeasure.setPower(0);
        backTapeMeasure.setPower(0);
    }
}
