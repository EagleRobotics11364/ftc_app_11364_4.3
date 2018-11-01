package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Land and Claim Depot: New Servo Values", group="League Meets")
public class AutoLandClaimNewServoValues extends AbstractLandingAuto {

    @Override
    void servoStartPosition() {
        robot.teamMarkerServo.setPosition(0.95);
    }

    @Override
    void servoDropTeamMarker() {
        robot.teamMarkerServo.setPosition(0.45);
    }
    @Override
    void doAfterLanding() {
        drive(-0.2, 0, 0, 500);
        drive(0,1,0,1500);
        servoDropTeamMarker();
        sleep(1000);
    }
}
