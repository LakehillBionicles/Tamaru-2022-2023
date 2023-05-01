package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;


@Config
@Autonomous(name = "TestVision")
public class ThreemaruTestVision extends ThreemaruAutoBase {
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        resetArm();
        resetDrive();
        telemetry.addData("beforeSignalSleeve", "yes");
        telemetry.update();
        scanSignalSleeve();
        telemetry.addData("AfterSignalSleeve", "yes?");
        telemetry.update();
        telemetryForVision();
        resetCamera();
        detectingCones();
        waitForStart();
        if (opModeIsActive()) {
            resetCamera();
            detectingCones();
            while (opModeIsActive()) {
                resetCamera();
                detectingCones();
                telemetry.addData("Color: ", ConeDetection.getBluePosition());
                telemetry.update();
                telemetry.addData("Color: ", ConeDetection.getRedPosition());
                telemetry.update();
            }
        }
    }
}