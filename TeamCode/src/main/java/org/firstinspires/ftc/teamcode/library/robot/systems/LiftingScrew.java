package org.firstinspires.ftc.teamcode.library.robot.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftingScrew implements Stoppable {
    final private DcMotor screw;
//    private Thread waitThread = new Thread(new Runnable() {
//        @Override
//        public void run() {
////                try {
////                    while(screw.isBusy())
////                    Thread.sleep(200);
////                } catch (InterruptedException e) {e.printStackTrace();}
//            try{
//                Thread.sleep(5000);
//                synchronized (screw) {
//                    screw.setPower(0);
//                    screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//            } catch (InterruptedException e) {e.printStackTrace();}
//
//        }
//    });
    public LiftingScrew(DcMotor screw) {
        this.screw = screw;
        ((DcMotorEx) screw).setTargetPositionTolerance(100);
    }

    public void up() {

            if (! screw.isBusy()) {
                this.screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.screw.setTargetPosition(-5200);
                this.screw.setPower(-1);
                this.screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

    }

    public void halfDown() {

            if (! screw.isBusy()) {
                this.screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.screw.setTargetPosition(5200/2);
                this.screw.setPower(1);
                this.screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

    }

    public void manualControl(double power) {
            if (power != 0) {
                this.screw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.screw.setPower(power);
            }
    }

    @Override
    public void stop() {
        this.screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
//    5200
}
