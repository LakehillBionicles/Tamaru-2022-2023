package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Config
@Autonomous(name = "JustParking")
public class ThreemaruParking extends ThreemaruAutoBase{
    @Override
    public void runOpMode()throws InterruptedException {
        super.runOpMode();
        robot.init(hardwareMap);
        resetArm();
        resetDrive();
        scanSignalSleeve();
        telemetryForVision();
        waitForStart();
        if(opModeIsActive()){
            encoderDrive(0.5, 28, 28);
            telemetry.addData("in loop?", "YES");
            telemetry.update();
            if(sideOfSleeve == 1){
                telemetry.addData("in loop?", "YES");
                encoderDrive(0.5,-9,9);
                encoderDrive(0.5,20,20);
            }else if(sideOfSleeve == 2){
                telemetry.addData("in loop2?", "YES");
                stop();
            }else{
                telemetry.addData("in loop else?", "YES");
                encoderDrive(0.5,9,-9);
                encoderDrive(0.5,20,20);
            }
            telemetry.update();
        }
    }
}