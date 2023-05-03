package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;


@Config
@Autonomous(name = "TestVision")
public class ThreemaruTestArmCodeVision extends ThreemaruAutoBase {
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        resetArm();
        resetDrive();
        scanSignalSleeve();
        telemetryForVision();
        resetCamera();
        detectingCones();
        while(!opModeIsActive()){
            telemetry.addData("sideOfSleeve", sideOfSleeve);
            telemetry.addData("blueColor: ", ConeDetection.getBluePosition());
            telemetry.addData("RedColor: ", ConeDetection.getRedPosition());
            telemetry.update();
        }
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                /*telemetry.addData("AfterWhile", "yes?");
                telemetry.update();
                //resetCamera();
                telemetry.addData("AfterresetCameraOpModeWhile", "yes?");
                telemetry.update();
                //detectingCones();
                 */
                telemetry.addData("Before colors", "yes?");
                telemetry.addData("sideOfSleeve", sideOfSleeve);
                telemetry.addData("blueColor: ", ConeDetection.getBluePosition());
                telemetry.addData("RedColor: ", ConeDetection.getRedPosition());
                telemetry.update();
            }
        }
    }
}
