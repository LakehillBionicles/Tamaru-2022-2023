package org.firstinspires.ftc.teamcode.Tamaru4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class TestEncoderDrive extends AutoBase4{

    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        resetDrive();

        waitForStart();
        encoderDrive(1, 24, 24, 5);
    }
}
