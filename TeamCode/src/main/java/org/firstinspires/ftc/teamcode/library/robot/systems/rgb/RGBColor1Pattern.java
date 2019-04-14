package org.firstinspires.ftc.teamcode.library.robot.systems.rgb;

public enum RGBColor1Pattern implements IndexableRGBPattern {
    END_TO_END_BLEND_TO_BLACK,
    LARSON_SCANNER,
    LIGHT_CHASE,
    HEARTBEAT_SLOW,
    HEARTBEAT_MEDIUM,
    HEARTBEAT_FAST,
    BREATH_SLOW,
    BREATH_FAST,
    SHOT,
    STROBE;
    
    @Override
    public int getServoInput() {
        return 1485 + ordinal() * 10;
    }
}
