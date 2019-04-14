package org.firstinspires.ftc.teamcode.library.robot.systems.rgb;

public enum RGBDualColorPattern implements IndexableRGBPattern {
    SPARKLE_COLOR1_ON_COLOR2,
    SPARKLE_COLOR2_ON_COLOR1,
    COLOR_GRADIENT,
    BEATS_PER_MINUTE,
    END_TO_END_BLEND_COLOR1_TO_COLOR2,
    END_TO_END_BLEND,
    NO_BLENDING,
    TWINKLES,
    COLOR_WAVES,
    SINELON;

    @Override
    public int getServoInput() {
        return 1685 + ordinal() * 10;
    }
}
