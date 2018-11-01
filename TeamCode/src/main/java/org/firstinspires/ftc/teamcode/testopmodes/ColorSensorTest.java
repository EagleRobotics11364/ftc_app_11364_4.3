package org.firstinspires.ftc.teamcode.testopmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

@TeleOp(name = "Color Sensor Test", group = "Test")
//@Disabled
public class ColorSensorTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;

    boolean enableLED;

    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            enableLED = true;
        } else if (gamepad1.b) {
            enableLED = false;
        }
        colorSensor.enableLed(enableLED);
        telemetry.addData("alpha", colorSensor.alpha());
        telemetry.addData("argb", colorSensor.argb());
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("led", enableLED);


    }
}
