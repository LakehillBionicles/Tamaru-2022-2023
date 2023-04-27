package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Config
@Autonomous(name = "TestAutoParkBlueCorner")
public class ThreemaruJustParkBlueCorner extends ThreemaruAutoBase{
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        resetArm();
        resetDrive();

        waitForStart();

        if (opModeIsActive()){
            encoderDrive(0.5, 17, 17);
            sleep(200);//Make sure it stops
            encoderDrive(0.5, 13, 13);
            if(sideOfSleeve == 1){
                rotate90Left();
                encoderDrive(0.5,13,13);
            }else if(sideOfSleeve == 2){
                stop();
            }else{
                rotate90Right();
                encoderDrive(0.5,13,13);
            }

        }
    }
}
