package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.robot.BaseRobot;

import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeBuffer;
import static org.firstinspires.ftc.teamcode.library.functions.MathOperations.rangeClip;
@TeleOp(name="State TeleOp", group="LT")
public class WorldTeleopJava extends OpMode {
    public BaseRobot robot;

    boolean slow = false;
    boolean reverse = false;

    boolean gamepad1StickButtonPressed = false;

    public void init() {
        robot = new BaseRobot(hardwareMap);
    }

    public void loop() {
        /*
        Driving Section
         */
       if(gamepad1.dpad_up) slow = false;
       else if (gamepad1.dpad_down) slow = true;

       if (gamepad1.right_stick_button) {
           if (!gamepad1StickButtonPressed) {
                gamepad1StickButtonPressed = true;
                slow = !slow;
           }
       } else gamepad1StickButtonPressed = false;


        double directions[] = {-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x};

        for (int i = 0; i < directions.length; i++) {
            directions[i] = rangeClip(directions[i], -1, 1);
            directions[i] = rangeBuffer(directions[i], -0.1, 0.1, 0);
            directions[i] *= (gamepad1.left_bumper & gamepad1.right_bumper) ? 0.50 : 1; // if slow is true, multiply by 0.25
        }

        robot.holonomic.runWithoutEncoder((reverse?-1:1)*directions[0], (reverse?-1:1)*directions[1], directions[2]);
//        telemetry.addData("Drive slow", slow);

        if (gamepad1.a) reverse = false;
        else if (gamepad1.b) reverse = true;



        /*
        Everything Else
         */
        if (gamepad2.right_bumper) {
            //control hanging screw
            robot.hangingScrew.manualControl(gamepad2.left_stick_y);
        } else {
            // intake arm raise/lower (with auto override)
            robot.intakeArmPivotController.controlFromGamepadInput(-gamepad2.left_stick_y, gamepad2.left_bumper);

            // intake arm extend/retract
            robot.intakeArmExtensionMotor.setPower(-gamepad2.left_stick_y);

            // mineral box pivot
            if (gamepad2.a) robot.collectionBoxPivotServo.setPosition(BaseRobot.INTAKE_BOX_POSITION_HOLDING);
            else if (gamepad2.b) robot.collectionBoxPivotServo.setPosition(BaseRobot.INTAKE_BOX_POSITION_COLLECTION_OR_DEPOSIT);
            else if (gamepad2.left_stick_button) {
                if (robot.intakeArmPivotController.getCurrentVoltage() > robot.intakeArmPivotController.getMIDPOINT_VERTICAL_VOLTAGE())
                    robot.collectionBoxPivotServo.setPosition(BaseRobot.INTAKE_BOX_POSITION_COLLECTION_OR_DEPOSIT);
                else robot.collectionBoxPivotServo.setPosition(BaseRobot.INTAKE_BOX_POSITION_HOLDING);
            }

            // change output direction
            if (gamepad2.dpad_left) robot.outputDirectionSwitcherServo.setPosition(BaseRobot.INTAKE_SILVER_OUTPUT_LEFT);
            else if (gamepad2.dpad_right) robot.outputDirectionSwitcherServo.setPosition(BaseRobot.INTAKE_SILVER_OUTPUT_RIGHT);

            // mineral collector speed
            if(gamepad2.right_trigger > 0.03) robot.collectorServo.setPosition(gamepad2.right_trigger);
            else if (gamepad2.left_trigger > 0.03) robot.collectorServo.setPosition(-(gamepad2.left_trigger * 0.5));
            else robot.collectorServo.setPosition(0);

            // yeet the team marker
            if (gamepad1.y) robot.teamMarkerServo.setPosition(BaseRobot.TEAM_MARKER_RELEASED); // y is for yeet
            else if (gamepad1.x) robot.teamMarkerServo.setPosition(BaseRobot.TEAM_MARKER_RETRACTED);
        }

        /*
        Intake Telemetry
         */
        telemetry.addData("Potentiometer output", robot.potentiometer.getVoltage());
        telemetry.addData("Potentiometer percent",robot.potentiometer.getMaxVoltage());
        telemetry.addData("Potentiometer angle", robot.potentiometer.getVoltage() * 270 / robot.potentiometer.getMaxVoltage());
        telemetry.addData("Lower proportion", robot.intakeArmPivotController.getLowerProportion());
        telemetry.addData("Upper proportion", robot.intakeArmPivotController.getUpperProportion());
        telemetry.addData("Full proportion", robot.intakeArmPivotController.getFullProportion());
        telemetry.addData("Intake power", robot.intakeArmPivotController.getCurrentMotorPower());
        telemetry.update();

    }



}
