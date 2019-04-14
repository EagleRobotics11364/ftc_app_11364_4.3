package org.firstinspires.ftc.teamcode.library.robot.systems.rgb;

public enum RGBPalettePattern implements IndexableRGBPattern {
    RAINBOW_RAINBOW_PALETTE,
    RAINBOW_PARTY_PALETTE,
    RAINBOW_OCEAN_PALETTE,
    RAINBOW_LAVA_PALETTE,
    RAINBOW_FOREST_PALETTE,
    RAINBOW_WITH_GLITTER,
    CONFETTI,
    SHOT_RED,
    SHOT_BLUE,
    SHOT_WHITE,
    SINELON_RAINBOW_PALLETE,
    SINELON_PARTY_PALLETE,
    SINELON_OCEAN_PALLETE,
    SINELON_LAVA_PALLETE,
    SINELON_FOREST_PALLETE,
    BEATS_PER_MINUTE_RAINBOW_PALETTE,
    BEATS_PER_MINUTE_PARTY_PALETTE,
    BEATS_PER_MINUTE_OCEAN_PALETTE,
    BEATS_PER_MINUTE_LAVA_PALETTE,
    BEATS_PER_MINUTE_FOREST_PALETTE,
    FIRE_MEDIUM,
    FIRE_LARGE,
    TWINKLES_RAINBOW_PALETTE,
    TWINKLES_PARTY_PALETTE,
    TWINKLES_OCEAN_PALETTE,
    TWINKLES_LAVA_PALETTE,
    TWINKLES_FOREST_PALETTE,
    COLOR_WAVES_RAINBOW_PALETTE,
    COLOR_WAVES_PARTY_PALETTE,
    COLOR_WAVES_OCEAN_PALETTE,
    COLOR_WAVES_LAVA_PALETTE,
    COLOR_WAVES_FOREST_PALETTE,
    LARSON_SCANNER_RED,
    LARSON_SCANNER_GRAY,
    LIGHT_CHASE_RED,
    LIGHT_CHASE_BLUE,
    LIGHT_CHASE_GRAY,
    HEARTBEAT_RED,
    HEARTBEAT_BLUE,
    HEARTBEAT_WHITE,
    HEARTBEAT_GRAY,
    BREATH_RED,
    BREATH_BLUE,
    BREATH_GRAY,
    STROBE_RED,
    STROBE_BLUE,
    STROBE_GOLD,
    STROBE_WHITE;

    @Override
    public int getServoInput() {
        return 1005 + ordinal() * 10;
    }
}