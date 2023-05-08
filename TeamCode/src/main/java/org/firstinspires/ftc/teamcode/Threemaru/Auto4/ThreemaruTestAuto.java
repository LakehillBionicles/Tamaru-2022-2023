package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@Config
@Autonomous(name = "TestAuto")
public class ThreemaruTestAuto extends ThreemaruAutoBase {
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        resetArm(); resetDrive();
        //scanSignalSleeve(); telemetryForVision();
        robot.servoHand1.setPosition(.25); robot.servoHand2.setPosition(.4); robot.servoExtend.setPosition(.5);
        robot.motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            PIDDrive(30, 0, 5000);
            //distDriveStar(-1, 3);
            //PIDDrive(-12,0, 5);
            /*PIDDrive(0, 90, 2);
            PIDDrive(10, 5);
            distDrivePort(1, 10);
            if (sideOfSleeve == 1) {
                PIDDrive(12, 5);
            } else if (sideOfSleeve == 2) {
                PIDDrive(-6, 5);
            } else {
                PIDDrive(-30, 5);
            }*/
        }
    }
}