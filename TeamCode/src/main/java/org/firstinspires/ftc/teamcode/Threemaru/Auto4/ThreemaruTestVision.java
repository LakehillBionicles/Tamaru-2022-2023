package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Config
@Autonomous(name = "TestVision")
public class ThreemaruTestVision extends ThreemaruAutoBase{
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        resetArm();
        resetDrive();
        scanSignalSleeve();
        telemetryForVision();
        waitForStart();
        while(opModeIsActive()){
            detectingCones();
        }
        stop();
    }
}