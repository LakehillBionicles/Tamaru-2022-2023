package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Config
@Autonomous(name = "JustParking")
public class ThreemaruParking extends ThreemaruAutoBase{
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        resetArm(); resetDrive();
        scanSignalSleeve(); telemetryForVision();
        telemetry.addData("sideOfSleeve",sideOfSleeve);
        waitForStart();
        if(opModeIsActive()){
            encoderDrive(0.5, 28, 28);
            if(sideOfSleeve == 1){
                encoderDrive(0.5,-9,9);
                encoderDrive(0.5,20,20);
            }else if(sideOfSleeve == 2){
                stop();
            }else{
                encoderDrive(0.5,9,-9);
                encoderDrive(0.5,20,20);
            }
        }
    }
}