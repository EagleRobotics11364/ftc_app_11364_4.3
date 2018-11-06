package org.firstinspires.ftc.teamcode.library.functions;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorOperations {
    private ColorOperations() {}

    public static float calculateHue(ColorSensor colorSensor) {
        return calculateHue(colorSensor.red(), colorSensor.green(), colorSensor.blue());
    }

    public static float calculateHue(int red, int green, int blue) {
        final double SCALE_FACTOR = 255;
        float hsvValues[] = new float[3];
        Color.RGBToHSV((int) (red * SCALE_FACTOR),
                (int) (green * SCALE_FACTOR),
                (int) (blue * SCALE_FACTOR),
                hsvValues);
        return hsvValues[0];
    }
}
