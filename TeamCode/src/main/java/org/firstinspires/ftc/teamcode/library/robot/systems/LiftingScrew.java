package org.firstinspires.ftc.teamcode.library.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftingScrew implements Stoppable {
    private DcMotor screw;
    private DcMotor rightScrew;

    public LiftingScrew(DcMotor screw) {
        this.screw = screw;

        this.rightScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void changeBy(int ticks) {
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        screw.setTargetPosition(screw.getCurrentPosition()+ticks);
        rightScrew.setPower(1);
    }

    @Override
    public void stop() {
        this.screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
