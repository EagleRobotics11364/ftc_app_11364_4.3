package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Land and Claim Depot", group="League Meets")
public class AutoLandClaim extends AbstractLandingAuto {
    @Override
    void doAfterLanding() {
        drive(-0.2, 0, 0, 500);
        drive(0,1,0,1500);
        robot.teamMarkerServo.setPosition(SERVO_POSITION_DOWN);
        sleep(1000);
    }
}
