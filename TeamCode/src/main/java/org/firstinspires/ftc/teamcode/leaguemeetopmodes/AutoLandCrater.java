package org.firstinspires.ftc.teamcode.leaguemeetopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Land and Park in Crater", group="League Meets")
public class AutoLandCrater extends AbstractLandingAuto {
    @Override
    void doAfterLanding() {
        drive(-0.2, 0, 0, 500);
        drive(0,0.7,0,1500);
        robot.teamMarkerServo.setPosition(SERVO_POSITION_DOWN);
        sleep(1000);
    }
}
