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
        scanSignalSleeve();
        telemetryForVision();
        while (!isStarted()) {
            telemetry.addData("Detected tag ID=%d", sideOfSleeve);
            telemetry.update();
        }

        if (opModeIsActive()){
            encoderDrive(0.5, 28, 28);
            if(sideOfSleeve == 1){
                telemetry.addData("in loop?", "YES");
                encoderDrive(0.5,-8,8);
                encoderDrive(0.5,13,13);
            }else if(sideOfSleeve == 2){
                telemetry.addData("in loop2?", "YES");
                stop();
            }else{
                telemetry.addData("in loop else?", "YES");
                encoderDrive(0.5,8,-8);
                encoderDrive(0.5,13,13);
            }
            telemetry.update();

        }
    }
}
