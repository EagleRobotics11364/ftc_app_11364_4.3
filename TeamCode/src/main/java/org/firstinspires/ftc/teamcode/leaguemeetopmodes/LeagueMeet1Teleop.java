/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "League Meet 1 Teleop", group = "League Meets")
//@Disabled
public class LeagueMeet1Teleop extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private LeagueMeet1Robot robot;
    private boolean slow = true;
    private boolean reverse = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot = new LeagueMeet1Robot(hardwareMap);
        robot.frontTapeMeasure.setMode(STOP_AND_RESET_ENCODER);
        robot.backTapeMeasure.setMode(STOP_AND_RESET_ENCODER);
        robot.frontTapeMeasure.setMode(RUN_USING_ENCODER);
        robot.backTapeMeasure.setMode(RUN_USING_ENCODER);

    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        // Drivetrain actions
//        if(!switchXY) robot.holonomic.runFromGamepadInput(gamepad1);
        if(reverse) robot.holonomic.runFromGamepadInputXYReversed(gamepad1, slow?0.50f:1f);
        else robot.holonomic.runFromGamepadInput(gamepad1, slow?0.50f:1f);

        if(gamepad1.a) reverse = false;
        else if (gamepad1.b) reverse = true;
        if(gamepad1.dpad_up) slow = false;
        else if (gamepad1.dpad_down) slow = true;

        //Tape measure spool actions
        controlTapeSpoolFromInput();

        //Update telemetry
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAllMotors();
    }

    private void controlTapeSpoolFromInput() {
        if(gamepad2.dpad_up) {
            // if up button on gamepad2 is pressed: move hook up
            robot.tapeMeasureHook.proportionalMove(1);
        } else if (gamepad2.dpad_down) {
            // if down button on gamepad2 is pressed: move hook down
            robot.tapeMeasureHook.proportionalMove(-1);
        } else if (gamepad2.left_bumper || gamepad2.right_bumper) {
            // if either bumper on gamepad2 is pressed: give custom control of each spool motor
            robot.frontTapeMeasure.setPower(-rangeClip(gamepad2.left_stick_y, -1, 1));
            robot.backTapeMeasure.setPower(-rangeClip(gamepad2.right_stick_y, -1, 1));
        } else {
            // if no condition above is met, set power to 0
            robot.frontTapeMeasure.setPower(0);
            robot.backTapeMeasure.setPower(0);
        }
        telemetry.addData("Front spool speed", robot.frontTapeMeasure.getPower());
        telemetry.addData("Back spool speed", robot.backTapeMeasure.getPower());
        telemetry.addData("Front encoder", robot.frontTapeMeasure.getCurrentPosition());
        telemetry.addData("Back encoder", robot.backTapeMeasure.getCurrentPosition());
    }

}